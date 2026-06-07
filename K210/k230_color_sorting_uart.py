import gc
import image
import time

from libs.PipeLine import ScopedTiming
from libs.Utils import *
from machine import FPIOA, Pin, UART
from media.display import *
from media.media import *
from media.sensor import *


# ============================================================
# Parameters that usually need real machine adjustment
# ============================================================

# K230 pins connected to STM32 USART3.
# Default STM32 side in this project: PB11 = USART3_RX.
# Connect K230 TX -> STM32 PB11, and connect GND together.
K230_UART_TX_PIN = 3
K230_UART_RX_PIN = 4
K230_UART_ID = UART.UART1
UART_BAUDRATE = 115200

# The result sent to STM32 uses display coordinates, not AI-channel coordinates.
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

# Camera AI channel.
OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

# Shape filtering for a 5 cm pure-color square.
MIN_PIXELS = 100
MIN_AREA = 100
MIN_ASPECT_RATIO = 0.65
MAX_ASPECT_RATIO = 1.45

# Send rate limit. STM32 does not need every camera frame.
SEND_PERIOD_MS = 50
SEND_ZERO_WHEN_LOST = True

debug_mode = 0


# ============================================================
# Color thresholds in LAB space.
# Tune these under your real light source and actual color blocks.
# ============================================================
thresholds = [
    (21, 82, 35, 127, 0, 82),      # 1 Red
    (33, 100, -46, -26, -61, 127), # 2 Green
    (34, 100, -20, 18, -41, -11),  # 3 Blue
    (65, 78, -10, -5, 38, 50),     # 4 Yellow
    (20, 50, 17, 37, -34, -14),    # 5 Purple
]

color_labels = ["Red", "Green", "Blue", "Yellow", "Purple"]
color_draw = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (128, 0, 128),
]
code_to_id = {1: 1, 2: 2, 4: 3, 8: 4, 16: 5}


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(K230_UART_RX_PIN, FPIOA.UART1_RXD)
    return UART(K230_UART_ID, UART_BAUDRATE)


def checksum(payload):
    total = 0
    for value in payload:
        total = (total + value) & 0xFF
    return total


def put_u16_le(data, value):
    value = int(max(0, min(65535, value)))
    data.append(value & 0xFF)
    data.append((value >> 8) & 0xFF)


def send_detection(uart, color_id, x, y, w, h):
    payload = bytearray()
    payload.append(int(max(0, min(255, color_id))))
    put_u16_le(payload, x)
    put_u16_le(payload, y)
    put_u16_le(payload, w)
    put_u16_le(payload, h)

    frame = bytearray([0xAA, 0x55])
    frame.extend(payload)
    frame.append(checksum(payload))
    uart.write(frame)


def select_best_blob(img):
    best = None
    best_score = 0

    for blob in img.find_blobs(
        thresholds,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
        margin=10,
    ):
        color_id = code_to_id.get(blob.code())
        if color_id is None:
            continue

        w = blob.w()
        h = blob.h()
        if h <= 0:
            continue

        ratio = w / h
        if ratio < MIN_ASPECT_RATIO or ratio > MAX_ASPECT_RATIO:
            continue

        score = blob.pixels()
        if score > best_score:
            best_score = score
            best = (color_id, blob)

    return best


def detection():
    print("k230 color sorting uart start")
    uart = init_uart()

    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    Display.init(
        Display.JD9852,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=1,
        osd_num=1,
        flag=Display.FLAG_ROTATION_90,
    )

    lcd_backlight = Pin(25, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    lcd_backlight.value(1)

    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    MediaManager.init()
    sensor.run()

    last_send = 0

    while True:
        with ScopedTiming("total", debug_mode > 0):
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                osd_img.clear()
                best = select_best_blob(rgb888p_img)
                now = time.ticks_ms()

                if best is not None:
                    color_id, blob = best
                    x, y, w, h = blob.rect()
                    x = int(x * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    y = int(y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    w = int(w * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    h = int(h * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    cx = x + w // 2
                    cy = y + h // 2

                    draw_color = color_draw[color_id - 1]
                    osd_img.draw_rectangle(x, y, w, h, color=draw_color)
                    osd_img.draw_cross(cx, cy, color=draw_color)
                    osd_img.draw_string_advanced(x, max(0, y - 32), 24, color_labels[color_id - 1], color=draw_color)

                    if time.ticks_diff(now, last_send) >= SEND_PERIOD_MS:
                        send_detection(uart, color_id, cx, cy, w, h)
                        last_send = now
                else:
                    if SEND_ZERO_WHEN_LOST and time.ticks_diff(now, last_send) >= SEND_PERIOD_MS:
                        send_detection(uart, 0, 0, 0, 0, 0)
                        last_send = now

                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()

            rgb888p_img = None


if __name__ == "__main__":
    detection()
