import gc

import image
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *
from machine import Pin  # 新增 LCD 背光控制

display_mode = "lcd"

# ============================================================
# 屏幕分辨率（与 JD9852 LCD 一致）
# ============================================================
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

debug_mode = 1

# ============================================================
# 颜色识别参数（同 multi_color）
# ============================================================
thresholds = [
    (21, 82, 35, 127, 0, 82),      # Red
    (33, 100, -46, -26, -61, 127), # Green
    (34, 100, -20, 18, -41, -11),  # Blue
    (65, 78, -10, -5, 38, 50),     # Yellow
    (20, 50, 17, 37, -34, -14),    # Purple
]

color_labels = ["Red", "Green", "Blue", "Yellow", "Purple"]
color_draw = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (128, 0, 128),
]
code_to_index = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4}


# ============================================================
# 主检测函数
# ============================================================
def detection():
    print("color_detect start")

    # ============================================================
    # 初始化 sensor
    # ============================================================
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)  # 显示通道
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)  # AI 通道
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    # ============================================================
    # LCD 显示初始化（修改为 JD9852）+ 背光
    # ============================================================
    Display.init(
        Display.JD9852,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=1,
        osd_num=1,
        flag=Display.FLAG_ROTATION_90
    )

    # LCD 背光控制（GPIO25）
    lcd_backlight = Pin(25, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    lcd_backlight.value(1)

    # ============================================================
    # 绑定 sensor 显示
    # ============================================================
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    # OSD 图像初始化
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    # media 初始化
    MediaManager.init()
    sensor.run()

    rgb888p_img = None

    # ============================================================
    # 主循环
    # ============================================================
    while True:
        with ScopedTiming("total", debug_mode > 0):
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                # 绘制结果
                osd_img.clear()
                for blob in rgb888p_img.find_blobs(
                    thresholds,
                    pixels_threshold=100,
                    area_threshold=100,
                    merge=True,
                    margin=10,
                ):
                    index = code_to_index.get(blob.code())
                    if index is None:
                        continue
                    x, y, w, h = blob.rect()
                    x = int(x * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    y = int(y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    w = int(w * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    h = int(h * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    osd_img.draw_rectangle(x, y, w, h, color=color_draw[index])
                    osd_img.draw_string_advanced(x, y - 40, 32, color_labels[index], color=color_draw[index])
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()
            rgb888p_img = None

    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    gc.collect()
    print("color_detect end")
    return 0


if __name__ == "__main__":
    detection()
