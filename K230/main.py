from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
import os, sys, gc
import ulab.numpy as np
import image
import time
from machine import UART, FPIOA

# K230 开发板上实际连接到 MSPM0 的 IO 编号
K230_UART_TX_IO = 3
K230_UART_RX_IO = 4

fpioa = FPIOA()

# 将物理 IO 映射到 UART1
fpioa.set_function(K230_UART_TX_IO, FPIOA.UART1_TXD)
fpioa.set_function(K230_UART_RX_IO, FPIOA.UART1_RXD)

print("UART1 TX IO:", fpioa.get_pin_num(FPIOA.UART1_TXD))
print("UART1 RX IO:", fpioa.get_pin_num(FPIOA.UART1_RXD))

uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=10
)


# 模型路径、标签名称、模型输入大小
kmodel_path="/sdcard/models/yolo11n_det_320.kmodel"
labels = {0: 'Target'}
model_input_size = [320, 320]
# 显示模式
display_mode = "st7701"
display_size = [800, 480]

rgb888p_size = [640, 360]

# 初始化PipeLine
pl = PipeLine(
    rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode
)

pl.create(sensor=Sensor(width=1920, height=1080))  # 创建PipeLine实例

display_size = pl.get_display_size()

# 初始化YOLO11实例
confidence_threshold = 0.8  # 置信度
nms_threshold = 0.45
yolo = YOLO11(
    task_type="detect",
    mode="video",
    kmodel_path=kmodel_path,
    labels=labels,
    rgb888p_size=rgb888p_size,
    model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold,
    nms_thresh=nms_threshold,
    max_boxes_num=50,
    debug_mode=0,
)
yolo.config_preprocess()

clock = time.clock()

# YOLO 推理图像的中心，不是 LCD 显示画面的中心
IMAGE_CENTER_X = rgb888p_size[0] // 2   # 320
IMAGE_CENTER_Y = rgb888p_size[1] // 2   # 180

# 中心死区，防止目标接近中心时数据不断跳动
DEAD_ZONE_X = 5
DEAD_ZONE_Y = 5

# UART 每 100 ms 发送一次，即 10 Hz
SEND_INTERVAL_MS = 100

last_send_ms = time.ticks_ms()
lost_count = 0
no_target_sent = False


def select_best_target(result):
    """
    YOLO 返回格式：
    result[0]：检测框列表，每个框为 [x1, y1, x2, y2]
    result[1]：类别 ID 列表
    result[2]：置信度列表

    返回置信度最高的目标。
    """
    boxes = result[0]
    class_ids = result[1]
    scores = result[2]

    if len(boxes) == 0:
        return None

    best_index = 0
    best_score = float(scores[0])

    for i in range(1, len(boxes)):
        current_score = float(scores[i])

        if current_score > best_score:
            best_index = i
            best_score = current_score

    box = boxes[best_index]

    return (
        int(box[0]),                  # x1
        int(box[1]),                  # y1
        int(box[2]),                  # x2
        int(box[3]),                  # y2
        int(class_ids[best_index]),   # 类别 ID
        best_score                    # 置信度
    )


try:
    while True:
        clock.tick()

        # 获取摄像头图像并进行 YOLO 推理
        img = pl.get_frame()
        res = yolo.run(img)

        # 在 LCD 上绘制识别框
        yolo.draw_result(res, pl.osd_img)

        # 选择一个最合适的目标
        target = select_best_target(res)

        if target is not None:
            x1, y1, x2, y2, class_id, confidence = target

            # 计算目标检测框中心
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # 计算目标中心相对于图像中心的偏差
            error_x = center_x - IMAGE_CENTER_X
            error_y = center_y - IMAGE_CENTER_Y

            # 设置中心死区
            if abs(error_x) <= DEAD_ZONE_X:
                error_x = 0

            if abs(error_y) <= DEAD_ZONE_Y:
                error_y = 0

            lost_count = 0
            no_target_sent = False

            now_ms = time.ticks_ms()

            # 限制 UART 发送频率
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                confidence_percent = int(confidence * 100)

                # 数据格式：
                # T,error_x,error_y,confidence\r\n
                message = "T,{},{},{}\r\n".format(
                    error_x,
                    error_y,
                    confidence_percent
                )

                uart.write(message)
                last_send_ms = now_ms

                print(
                    "TX:",
                    message,
                    "box:",
                    x1,
                    y1,
                    x2,
                    y2,
                    "center:",
                    center_x,
                    center_y
                )

        else:
            # 单帧没检测到目标时，不立即发送丢失
            lost_count += 1

            # 连续丢失 3 帧后发送一次 N
            if lost_count >= 3 and not no_target_sent:
                uart.write("N\r\n")
                no_target_sent = True

                print("TX: N")

        # 读取 MSPM0 回环数据
        received = uart.read()

        if received:
            print("RX:", received)

        # 显示画面
        pl.show_image()

        # 回收内存
        gc.collect()

        print("FPS:", clock.fps())

except KeyboardInterrupt:
    print("程序停止")

finally:
    # 释放硬件和模型资源
    yolo.deinit()
    pl.destroy()
    uart.deinit()
    gc.collect()

    print("资源已释放")
