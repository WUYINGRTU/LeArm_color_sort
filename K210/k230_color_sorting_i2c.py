import gc
import image
import time

from libs.PipeLine import ScopedTiming
from libs.Utils import *
from machine import FPIOA, I2C_Slave, Pin
from media.display import *
from media.media import *
from media.sensor import *


# ============================================================
# I2C output
# ============================================================

# STM32 uses 7-bit address 0x32 in Hiwonder/Peripherals/Inc/wonder_mv.h.
I2C_SLAVE_ADDR = 0x32

# K230 I2C_Slave.list() returns real slave device IDs, for example [2].
# This value is the index in that list, not the hardware ID itself.
I2C_SLAVE_LIST_INDEX = 0
I2C_REG_COLOR = 0x00
I2C_MEM_SIZE = 64

# K230 IOMUX mapping from the manual:
# pin 11 -> I2C2_SCL, pin 12 -> I2C2_SDA, pull-up enabled.
K230_I2C_SCL_PIN = 11
K230_I2C_SDA_PIN = 12

# Optional notify pin from the manual. This project does not require STM32
# to read it, but keeping it high matches the demo board behavior.
K230_NOTIFY_PIN = 14

# Result layout at register 0x00:
# [0] id, [1..2] x, [3..4] y, [5..6] w, [7..8] h, little endian.
color_data = bytearray(9)


# ============================================================
# Vision parameters
# ============================================================

DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

MIN_PIXELS = 100
MIN_AREA = 100

UPDATE_PERIOD_MS = 50
CLEAR_WHEN_LOST = True
PRINT_PERIOD_MS = 300
PRINT_NONE_PERIOD_MS = 1000
debug_mode = 0


# Tune these LAB thresholds under your real light source and actual color blocks.
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


def put_u16_le(data, offset, value):
    value = int(max(0, min(65535, value)))
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def set_color_data(color_id, x, y, w, h):
    color_data[0] = int(max(0, min(255, color_id)))
    put_u16_le(color_data, 1, x)
    put_u16_le(color_data, 3, y)
    put_u16_le(color_data, 5, w)
    put_u16_le(color_data, 7, h)


def init_i2c_slave():
    fpioa = FPIOA()
    fpioa.set_function(K230_NOTIFY_PIN, FPIOA.GPIO14)
    notify_pin = Pin(K230_NOTIFY_PIN, Pin.OUT, pull=Pin.PULL_UP, drive=7)
    notify_pin.value(1)

    fpioa.set_function(K230_I2C_SCL_PIN, FPIOA.IIC2_SCL, pu=1)
    fpioa.set_function(K230_I2C_SDA_PIN, FPIOA.IIC2_SDA, pu=1)

    device_ids = I2C_Slave.list()
    print("available i2c slave ids:", device_ids)
    if not device_ids:
        raise RuntimeError("no I2C slave device found; check K230 firmware I2C slave support")

    i2c_slave = I2C_Slave(device_ids[I2C_SLAVE_LIST_INDEX], addr=I2C_SLAVE_ADDR, mem_size=I2C_MEM_SIZE)
    set_color_data(0, 0, 0, 0, 0)
    i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
    return i2c_slave


def detection():
    print("k230 color sorting i2c start")
    i2c_slave = init_i2c_slave()

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

    last_update = 0
    last_print = 0
    last_none_print = 0

    while True:
        with ScopedTiming("total", debug_mode > 0):
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                osd_img.clear()
                now = time.ticks_ms()
                detect = False

                for blob in rgb888p_img.find_blobs(
                    thresholds,
                    pixels_threshold=MIN_PIXELS,
                    area_threshold=MIN_AREA,
                    merge=True,
                    margin=10,
                ):
                    color_id = code_to_id.get(blob.code())
                    if color_id is None:
                        continue

                    detect = True
                    x, y, w, h = blob.rect()
                    x = int(x * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    y = int(y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    w = int(w * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    h = int(h * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    cx = x + w // 2
                    cy = y + h // 2

                    draw_color = color_draw[color_id - 1]
                    label = "%s %d,%d" % (color_labels[color_id - 1], cx, cy)
                    osd_img.draw_rectangle(x, y, w, h, color=draw_color)
                    osd_img.draw_cross(cx, cy, color=draw_color)
                    osd_img.draw_string_advanced(x, max(0, y - 32), 24, label, color=draw_color)

                    if time.ticks_diff(now, last_update) >= UPDATE_PERIOD_MS:
                        set_color_data(color_id, cx, cy, w, h)
                        i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
                        last_update = now

                    if time.ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
                        print(
                            "DETECT id=%d color=%s cx=%d cy=%d w=%d h=%d"
                            % (color_id, color_labels[color_id - 1], cx, cy, w, h)
                        )
                        last_print = now

                if not detect:
                    if CLEAR_WHEN_LOST and time.ticks_diff(now, last_update) >= UPDATE_PERIOD_MS:
                        set_color_data(0, 0, 0, 0, 0)
                        i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
                        last_update = now

                    if time.ticks_diff(now, last_none_print) >= PRINT_NONE_PERIOD_MS:
                        print("DETECT none")
                        last_none_print = now

                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()

            rgb888p_img = None


if __name__ == "__main__":
    detection()
