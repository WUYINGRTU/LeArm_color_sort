# K230 Eye-in-Hand Color Sorting Calibration

本工程当前采用“眼在手上 + 固定拍照姿态 + I2C 视觉数据读取 + 一次视觉定位抓取”的第一版方案。

## 接线

- K230 I2C SCL -> STM32 PB6 / I2C1_SCL
- K230 I2C SDA -> STM32 PB7 / I2C1_SDA
- K230 GND -> STM32 GND
- I2C 电平：3.3V，SCL/SDA 需要上拉电阻
- I2C 从机地址：`0x32`
- STM32 I2C1：100 kHz
- K230 脚本参数位置：`K210/k230_color_sorting_i2c.py`
- STM32 标定参数位置：`STM32/LeArm/Core/Src/main.c`

注意：K230 CanMV 的 `machine.I2C_Slave` 需要固件启用 I2C 从机支持。如果运行脚本时 `I2C_Slave.list()` 为空或报错，需要换用带 I2C slave 配置的固件。

## K230 需要调的参数

在 `K210/k230_color_sorting_i2c.py` 中修改：

- `I2C_SLAVE_ID_INDEX`：K230 实际启用的 I2C 从机通道编号，可通过脚本打印的 `I2C_Slave.list()` 确认。
- `I2C_SLAVE_ADDR`：默认 `0x32`，需要和 STM32 `wonder_mv.h` 中的 `WONDERMV_ADDR` 一致。
- `thresholds`：红、绿、蓝、黄、紫的 LAB 阈值，需要在你的灯光和色块下实测。
- `MIN_PIXELS` / `MIN_AREA`：过滤太小的误检。
- `MIN_ASPECT_RATIO` / `MAX_ASPECT_RATIO`：过滤不像正方形的色块。

## STM32 需要实测的参数

在 `STM32/LeArm/Core/Src/main.c` 中修改：

- `VISION_CAPTURE_X_CM`
- `VISION_CAPTURE_Y_CM`
- `VISION_CAPTURE_Z_CM`

这是识别前机械臂固定移动到的拍照姿态。摄像头最好尽量垂直看桌面，且色块能完整出现在画面里。

- `VISION_X_CM_PER_PIXEL`
- `VISION_Y_CM_PER_PIXEL`

这是像素偏差到机械臂坐标偏差的比例。测试方法：

1. 机械臂移动到拍照姿态。
2. 放色块并记录 K230 输出的中心像素 `(cx, cy)`。
3. 把色块在桌面上沿机械臂 x 或 y 方向移动一个已知距离，比如 2 cm。
4. 记录像素变化量。
5. 比例 = 实际移动距离 cm / 像素变化量。
6. 如果机械臂移动方向反了，把对应比例改成负数。

- `CAMERA_TO_CLAW_X_OFFSET_CM`
- `CAMERA_TO_CLAW_Y_OFFSET_CM`

这是摄像头中心与夹爪中心的安装偏移。测试方法：

1. 让色块位于 K230 画面中心。
2. 控制机械臂下探到抓取高度。
3. 看夹爪中心相对色块中心偏了多少 cm。
4. 把这个偏差填入两个 offset。

- `VISION_APPROACH_Z_CM`
- `VISION_GRAB_Z_CM`

这是抓取前的安全高度和下探抓取高度。先用较高的 `VISION_GRAB_Z_CM` 测试，再逐步降低，避免撞桌面。

- `CLAW_OPEN_ANGLE`
- `CLAW_CLOSE_ANGLE`

这是夹爪打开和夹紧 5 cm 色块时的角度。

- `PLACE_*_X_CM`
- `PLACE_*_Y_CM`
- `PLACE_Z_CM`

这是不同颜色放置点。若放置区固定，直接写死坐标最稳。若放置区会移动，后续再增加 K230 对放置区标记的识别。

## 当前动作流程

1. 机械臂回到固定拍照姿态。
2. K230 识别最大且近似正方形的纯色色块。
3. K230 将颜色 ID、中心像素和检测框大小写入 I2C 寄存器 `0x00`。
4. STM32 通过 I2C 读取 9 字节结果，并将像素中心换算成抓取坐标。
5. 机械臂移动到色块上方。
6. 机械臂下探并夹取。
7. 机械臂抬起。
8. 按颜色移动到对应放置点。
9. 打开夹爪并回到拍照姿态。
