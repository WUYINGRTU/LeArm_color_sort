import gc
import time

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from machine import FPIOA, I2C_Slave, Pin
from media.display import *
from media.media import *
from media.sensor import *


# ============================================================
# I2C result protocol
# ============================================================

I2C_SLAVE_ADDR = 0x32
I2C_SLAVE_LIST_INDEX = 0
I2C_MEM_SIZE = 128

K230_I2C_SCL_PIN = 11
K230_I2C_SDA_PIN = 12
K230_NOTIFY_PIN = 14

COLOR_LEGACY_REG = 0x00
RESULT_ITEM_SIZE = 9
LEGACY_COLOR_OFFSET = 0
LEGACY_NUMBER_BASE_OFFSET = 9
LEGACY_NUMBER_COUNT = 5
LEGACY_PACKET_SIZE = RESULT_ITEM_SIZE * (1 + LEGACY_NUMBER_COUNT)

COLOR_REGS = {
    1: 0x10,  # RED
    2: 0x19,  # GREEN
    3: 0x22,  # BLUE
}

NUMBER_REGS = {
    1: 0x40,
    2: 0x49,
    3: 0x52,
    4: 0x5B,
    5: 0x64,
}

RESULT_REGS = [COLOR_LEGACY_REG] + list(COLOR_REGS.values()) + list(NUMBER_REGS.values())
I2C_WRITE_REGS = [COLOR_LEGACY_REG]
result_data = {}


# ============================================================
# Display and model parameters
# ============================================================

DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

ROOT_PATH = "/sdcard/mp_deployment_source2/"
CONFIG_PATH = "/sdcard/mp_deployment_source2/deploy_config.json"

MIN_PIXELS = 200
MIN_AREA = 200
MERGE_MARGIN = 15

UPDATE_PERIOD_MS = 50
PRINT_PERIOD_MS = 500

debug_mode = 0

# Use this when the display/OSD coordinate system and the color detection
# coordinate system do not match on the real K230 screen.
COLOR_COORD_MODE = "normal"


# Keep the original red/green/blue LAB thresholds from Multiple color recognition.py.
color_thresholds = [
    (0, 100, 29, 127, 3, 127),       # RED
    (0, 100, -128, -18, 1, 127),     # GREEN
    (19, 58, -22, 31, -50, -12),     # BLUE
]

color_names = ["RED", "GREEN", "BLUE"]
color_draw = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
]


def put_u16_le(data, offset, value):
    value = int(max(0, min(65535, value)))
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def set_result_slot(data, offset, target_id, cx, cy, w, h):
    data[offset] = int(max(0, min(255, target_id)))
    put_u16_le(data, offset + 1, cx)
    put_u16_le(data, offset + 3, cy)
    put_u16_le(data, offset + 5, w)
    put_u16_le(data, offset + 7, h)


def set_result_data(reg, target_id, cx, cy, w, h):
    set_result_slot(result_data[reg], 0, target_id, cx, cy, w, h)


def set_legacy_color_data(target_id, cx, cy, w, h):
    set_result_slot(result_data[COLOR_LEGACY_REG], LEGACY_COLOR_OFFSET, target_id, cx, cy, w, h)


def set_legacy_number_data(number_id, cx, cy, w, h):
    if number_id < 1 or number_id > LEGACY_NUMBER_COUNT:
        return
    offset = LEGACY_NUMBER_BASE_OFFSET + (number_id - 1) * RESULT_ITEM_SIZE
    set_result_slot(result_data[COLOR_LEGACY_REG], offset, number_id, cx, cy, w, h)


def clear_all_results():
    for data in result_data.values():
        for i in range(len(data)):
            data[i] = 0


def init_i2c_slave():
    global result_data

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

    result_data = {}
    for reg in RESULT_REGS:
        if reg == COLOR_LEGACY_REG:
            result_data[reg] = bytearray(LEGACY_PACKET_SIZE)
        else:
            result_data[reg] = bytearray(RESULT_ITEM_SIZE)

    i2c_slave = I2C_Slave(device_ids[I2C_SLAVE_LIST_INDEX], addr=I2C_SLAVE_ADDR, mem_size=I2C_MEM_SIZE)
    clear_all_results()
    write_all_results(i2c_slave)
    return i2c_slave


def write_all_results(i2c_slave):
    for reg in I2C_WRITE_REGS:
        i2c_slave.writeto_mem(reg, result_data[reg])


def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)
    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])
    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio


def read_deploy_config(config_path):
    with open(config_path, "r") as json_file:
        return ujson.load(json_file)


def scale_rect_to_display(x1, y1, x2, y2):
    x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
    y = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
    w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
    h = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
    cx = x + w // 2
    cy = y + h // 2
    return x, y, w, h, cx, cy


def clamp_coord(value, max_value):
    return int(max(0, min(max_value, value)))


def transform_color_coord_for_arm(cx, cy, w, h):
    if COLOR_COORD_MODE == "rot90":
        send_x = cy * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_y = (DISPLAY_WIDTH - cx) * DISPLAY_HEIGHT // DISPLAY_WIDTH
        send_w = h * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_h = w * DISPLAY_HEIGHT // DISPLAY_WIDTH
    elif COLOR_COORD_MODE == "rot270":
        send_x = (DISPLAY_HEIGHT - cy) * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_y = cx * DISPLAY_HEIGHT // DISPLAY_WIDTH
        send_w = h * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_h = w * DISPLAY_HEIGHT // DISPLAY_WIDTH
    elif COLOR_COORD_MODE == "mirror_x":
        send_x = DISPLAY_WIDTH - cx
        send_y = cy
        send_w = w
        send_h = h
    elif COLOR_COORD_MODE == "mirror_y":
        send_x = cx
        send_y = DISPLAY_HEIGHT - cy
        send_w = w
        send_h = h
    elif COLOR_COORD_MODE == "swap_xy":
        send_x = cy * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_y = cx * DISPLAY_HEIGHT // DISPLAY_WIDTH
        send_w = h * DISPLAY_WIDTH // DISPLAY_HEIGHT
        send_h = w * DISPLAY_HEIGHT // DISPLAY_WIDTH
    else:
        send_x = cx
        send_y = cy
        send_w = w
        send_h = h

    send_x = clamp_coord(send_x, DISPLAY_WIDTH)
    send_y = clamp_coord(send_y, DISPLAY_HEIGHT)
    send_w = clamp_coord(send_w, DISPLAY_WIDTH)
    send_h = clamp_coord(send_h, DISPLAY_HEIGHT)
    return send_x, send_y, send_w, send_h


def label_to_number(labels, class_id):
    label = str(labels[class_id]).strip()
    try:
        number_id = int(label)
    except Exception:
        number_id = int(class_id) + 1

    if number_id < 1 or number_id > 5:
        return 0
    return number_id


def update_color_results(color_img, osd_img):
    best_colors = {}
    best_legacy_color = None

    for i in range(3):
        color_id = i + 1
        blobs = color_img.find_blobs(
            [color_thresholds[i]],
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA,
            merge=True,
            margin=MERGE_MARGIN,
        )

        for blob in blobs:
            x, y, w, h = blob.rect()
            cx = x + w // 2
            cy = y + h // 2
            send_cx, send_cy, send_w, send_h = transform_color_coord_for_arm(cx, cy, w, h)
            draw_color = color_draw[i]

            osd_img.draw_rectangle(x, y, w, h, color=draw_color)
            osd_img.draw_cross(cx, cy, color=draw_color)
            osd_img.draw_string_advanced(x, max(0, y - 28), 24, "%s %d,%d" % (color_names[i], cx, cy), color=draw_color)
            osd_img.draw_string_advanced(x, min(DISPLAY_HEIGHT - 24, y + h + 2), 20, "SEND %d,%d" % (send_cx, send_cy), color=draw_color)

            pixels = blob.pixels()
            if color_id not in best_colors or pixels > best_colors[color_id][0]:
                best_colors[color_id] = (pixels, cx, cy, w, h, send_cx, send_cy, send_w, send_h)

            if best_legacy_color is None or pixels > best_legacy_color[0]:
                best_legacy_color = (pixels, color_id, send_cx, send_cy, send_w, send_h)

    if best_legacy_color is not None:
        _, color_id, send_cx, send_cy, send_w, send_h = best_legacy_color
        set_legacy_color_data(color_id, send_cx, send_cy, send_w, send_h)

    for color_id, reg in COLOR_REGS.items():
        if color_id in best_colors:
            _, raw_cx, raw_cy, raw_w, raw_h, send_cx, send_cy, send_w, send_h = best_colors[color_id]
            set_result_data(reg, color_id, send_cx, send_cy, send_w, send_h)

    return best_colors


def update_number_results(det_boxes, labels, color_four, osd_img):
    best_numbers = {}

    if not det_boxes:
        return best_numbers

    for det_box in det_boxes:
        class_id = int(det_box[0])
        score = float(det_box[1])
        number_id = label_to_number(labels, class_id)
        if number_id == 0:
            continue

        x, y, w, h, cx, cy = scale_rect_to_display(det_box[2], det_box[3], det_box[4], det_box[5])
        draw_color = color_four[class_id][1:]
        text = "N%d %.2f %d,%d" % (number_id, score, cx, cy)

        osd_img.draw_rectangle(x, y, w, h, color=draw_color)
        osd_img.draw_cross(cx, cy, color=draw_color)
        osd_img.draw_string_advanced(x, min(DISPLAY_HEIGHT - 24, y + h + 2), 24, text, color=draw_color)

        if number_id not in best_numbers or score > best_numbers[number_id][0]:
            best_numbers[number_id] = (score, cx, cy, w, h)

    for number_id, reg in NUMBER_REGS.items():
        if number_id in best_numbers:
            _, cx, cy, w, h = best_numbers[number_id]
            set_legacy_number_data(number_id, cx, cy, w, h)
            set_result_data(reg, number_id, cx, cy, w, h)

    return best_numbers


def draw_control_overlay(osd_img):
    center_x = DISPLAY_WIDTH // 2
    center_y = DISPLAY_HEIGHT // 2

    osd_img.draw_cross(center_x, center_y, color=(255, 255, 255))
    osd_img.draw_string_advanced(
        0,
        0,
        20,
        "CENTER %d,%d MODE %s" % (center_x, center_y, COLOR_COORD_MODE),
        color=(255, 255, 255),
    )


def print_debug(best_colors, best_numbers):
    color_text = []
    number_text = []

    for color_id in sorted(best_colors.keys()):
        _, raw_cx, raw_cy, raw_w, raw_h, send_cx, send_cy, send_w, send_h = best_colors[color_id]
        color_text.append(
            "%s raw=%d,%d send=%d,%d w=%d h=%d"
            % (color_names[color_id - 1], raw_cx, raw_cy, send_cx, send_cy, send_w, send_h)
        )

    for number_id in sorted(best_numbers.keys()):
        score, cx, cy, w, h = best_numbers[number_id]
        number_text.append("N%d:%.2f,%d,%d,%d,%d" % (number_id, score, cx, cy, w, h))

    print("DETECT color=[%s] number=[%s]" % (";".join(color_text), ";".join(number_text)))


def detection():
    print("k230 color number i2c start")
    i2c_slave = init_i2c_slave()

    deploy_conf = read_deploy_config(CONFIG_PATH)
    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    anchors = []
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides = [8, 16, 32]
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    kpu = nn.kpu()
    kpu.load_kmodel(ROOT_PATH + kmodel_name)

    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH],
        [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]],
    )

    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
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

    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    MediaManager.init()
    sensor.run()

    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    last_update = 0
    last_print = 0

    while True:
        with ScopedTiming("total", debug_mode > 0):
            color_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                osd_img.clear()
                clear_all_results()

                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()

                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                    del out_data
                    results.append(result)

                det_boxes = aicube.anchorbasedet_post_process(
                    results[0],
                    results[1],
                    results[2],
                    kmodel_frame_size,
                    frame_size,
                    strides,
                    num_classes,
                    confidence_threshold,
                    nms_threshold,
                    anchors,
                    nms_option,
                )

                best_colors = update_color_results(color_img, osd_img)
                best_numbers = update_number_results(det_boxes, labels, color_four, osd_img)
                draw_control_overlay(osd_img)

                now = time.ticks_ms()
                if time.ticks_diff(now, last_update) >= UPDATE_PERIOD_MS:
                    write_all_results(i2c_slave)
                    last_update = now

                if time.ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
                    print_debug(best_colors, best_numbers)
                    last_print = now

                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()

            rgb888p_img = None
            color_img = None


if __name__ == "__main__":
    detection()
