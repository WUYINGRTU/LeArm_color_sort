import time

from machine import FPIOA, I2C_Slave, Pin
from media.display import *
from media.media import *
from media.sensor import *


# =================================================
# I2C output
# =================================================

I2C_SLAVE_ADDR = 0x32
I2C_SLAVE_LIST_INDEX = 0
I2C_REG_COLOR = 0x00
I2C_MEM_SIZE = 64

K230_I2C_SCL_PIN = 11
K230_I2C_SDA_PIN = 12
K230_NOTIFY_PIN = 14

color_data = bytearray(9)


# =================================================
# Vision parameters
# =================================================

DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

MIN_PIXELS = 200
MIN_AREA = 200
MERGE_MARGIN = 15

UPDATE_PERIOD_MS = 50
PRINT_PERIOD_MS = 300
PRINT_NONE_PERIOD_MS = 1000


# Keep the original red/green/blue LAB thresholds from this demo.
thresholds = [
    (0, 100, 29, 127, 3, 127),       # RED
    (0, 100, -128, -18, 1, 127),     # GREEN
    (19, 58, -22, 31, -50, -12),     # BLUE
]

colors1 = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
colors2 = ["RED", "GREEN", "BLUE"]

lcd_backlight = None


def put_u16_le(data, offset, value):
    value = int(max(0, min(65535, value)))
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def set_color_data(color_id, cx, cy, w, h):
    color_data[0] = int(max(0, min(255, color_id)))
    put_u16_le(color_data, 1, cx)
    put_u16_le(color_data, 3, cy)
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
        raise RuntimeError("no I2C slave device found")

    i2c_slave = I2C_Slave(device_ids[I2C_SLAVE_LIST_INDEX], addr=I2C_SLAVE_ADDR, mem_size=I2C_MEM_SIZE)
    set_color_data(0, 0, 0, 0, 0)
    i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
    return i2c_slave


def init_display():
    global lcd_backlight
    Display.init(
        Display.JD9852,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True,
    )

    lcd_backlight = Pin(25, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    lcd_backlight.value(1)

    print("JD9852 LCD init: %dx%d" % (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    return DISPLAY_WIDTH, DISPLAY_HEIGHT


def deinit_display():
    Display.deinit()
    print("display deinit")


def get_blob_info(blob):
    x, y, w, h = blob[0], blob[1], blob[2], blob[3]
    cx, cy = blob[5], blob[6]
    return x, y, w, h, cx, cy


if __name__ == "__main__":
    i2c_slave = init_i2c_slave()
    width, height = init_display()

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=width, height=height)
    sensor.set_pixformat(Sensor.RGB565)

    MediaManager.init()
    sensor.run()

    clock = time.clock()
    last_update = 0
    last_print = 0
    last_none_print = 0

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()
            best_target = None
            best_pixels = -1

            for i in range(3):
                blobs = img.find_blobs(
                    [thresholds[i]],
                    pixels_threshold=MIN_PIXELS,
                    area_threshold=MIN_AREA,
                    merge=True,
                    margin=MERGE_MARGIN,
                )

                for blob in blobs:
                    x, y, w, h, cx, cy = get_blob_info(blob)
                    img.draw_rectangle(blob[0:4], thickness=4, color=colors1[i])
                    img.draw_cross(cx, cy, thickness=2, color=colors1[i])
                    img.draw_string_advanced(x, y - 35, 30, colors2[i], color=colors1[i])

                    pixels = blob.pixels()
                    if pixels > best_pixels:
                        best_pixels = pixels
                        best_target = (i + 1, colors2[i], cx, cy, w, h)

            if best_target:
                color_id, color_name, cx, cy, w, h = best_target
                if time.ticks_diff(now, last_update) >= UPDATE_PERIOD_MS:
                    set_color_data(color_id, cx, cy, w, h)
                    i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
                    last_update = now

                if time.ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
                    print(
                        "DETECT id=%d color=%s cx=%d cy=%d w=%d h=%d"
                        % (color_id, color_name, cx, cy, w, h)
                    )
                    last_print = now
            else:
                if time.ticks_diff(now, last_update) >= UPDATE_PERIOD_MS:
                    set_color_data(0, 0, 0, 0, 0)
                    i2c_slave.writeto_mem(I2C_REG_COLOR, color_data)
                    last_update = now

                if time.ticks_diff(now, last_none_print) >= PRINT_NONE_PERIOD_MS:
                    print("DETECT none")
                    last_none_print = now

            img.draw_string_advanced(
                0,
                0,
                30,
                "FPS: %.3f" % clock.fps(),
                color=(255, 255, 255),
            )

            Display.show_image(img)

    except KeyboardInterrupt:
        print("user interrupted")
    except Exception as e:
        print("program stopped:", e)
    finally:
        sensor.stop()
        deinit_display()
