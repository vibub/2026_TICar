import gc
import os
import sys
import time


K230_UART_TX_IO = 3
K230_UART_RX_IO = 4

VISION_SIZE = (640, 360)
DISPLAY_SIZE = (800, 480)
MODEL_INPUT_SIZE = [320, 320]
MODEL_PATH = "/sdcard/models/yolo11n_det_320.kmodel"
LABELS = {0: "Target"}

CONFIDENCE_THRESHOLD = 0.8
NMS_THRESHOLD = 0.45
DEAD_ZONE_X = 5
DEAD_ZONE_Y = 5
SEND_INTERVAL_MS = 100
DEBUG_PRINT = True


fpioa = None
uart = None
pipeline = None
yolo = None


def select_best_target(result):
    boxes = result[0]
    scores = result[2]

    if len(boxes) == 0:
        return None

    best_index = 0
    best_score = float(scores[0])

    for index in range(1, len(boxes)):
        score = float(scores[index])
        if score > best_score:
            best_index = index
            best_score = score

    box = boxes[best_index]

    # YOLO11 video模式返回显示坐标系中的[x, y, width, height]。
    return (
        int(box[0]),
        int(box[1]),
        int(box[2]),
        int(box[3]),
        best_score
    )


def display_box_to_vision_bounds(target):
    x, y, width, height, confidence = target
    scale_x = VISION_SIZE[0] / DISPLAY_SIZE[0]
    scale_y = VISION_SIZE[1] / DISPLAY_SIZE[1]

    x1 = int(x * scale_x)
    y1 = int(y * scale_y)
    x2 = int((x + width) * scale_x)
    y2 = int((y + height) * scale_y)

    x1 = max(0, min(VISION_SIZE[0] - 1, x1))
    y1 = max(0, min(VISION_SIZE[1] - 1, y1))
    x2 = max(x1 + 1, min(VISION_SIZE[0], x2))
    y2 = max(y1 + 1, min(VISION_SIZE[1], y2))

    return x1, y1, x2, y2, confidence


def apply_dead_zone(error_x, error_y):
    if abs(error_x) <= DEAD_ZONE_X:
        error_x = 0

    if abs(error_y) <= DEAD_ZONE_Y:
        error_y = 0

    return error_x, error_y


try:
    print("S00: 导入K230模块")
    from libs.YOLO import YOLO11
    from machine import UART, FPIOA
    from target_geometry import locate_target_center
    from vision_pipeline import VisionPipeline
    print("S01: 模块导入完成")

    print("S10: 初始化UART")
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_IO, FPIOA.UART1_TXD)
    fpioa.set_function(K230_UART_RX_IO, FPIOA.UART1_RXD)

    uart = UART(
        UART.UART1,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=10
    )
    print("S11: UART初始化完成")

    print("S20: 初始化摄像头和显示")
    pipeline = VisionPipeline(
        vision_size=VISION_SIZE,
        display_size=DISPLAY_SIZE
    )
    pipeline.create()
    print("S21: 摄像头和显示初始化完成")

    print("S30: 加载YOLO模型")
    yolo = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=MODEL_PATH,
        labels=LABELS,
        rgb888p_size=list(VISION_SIZE),
        model_input_size=MODEL_INPUT_SIZE,
        display_size=list(pipeline.display_size),
        conf_thresh=CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=50,
        debug_mode=0
    )
    print("S31: YOLO模型加载完成")

    print("S40: 配置YOLO预处理")
    yolo.config_preprocess()
    print("S41: YOLO预处理配置完成")

    if DEBUG_PRINT:
        print("K230视觉程序启动")
        print("UART1: 115200 8N1, TX IO", K230_UART_TX_IO,
              "RX IO", K230_UART_RX_IO)
        print("视觉坐标:", VISION_SIZE, "显示坐标:", DISPLAY_SIZE)

    image_center_x = VISION_SIZE[0] // 2
    image_center_y = VISION_SIZE[1] // 2
    last_send_ms = time.ticks_ms()
    previous_center = None
    gc_counter = 0

    while True:
        os.exitpoint()

        ai_frame, ai_array = pipeline.capture_ai_frame()
        os.exitpoint()
        result = yolo.run(ai_array)

        pipeline.clear_osd()
        yolo.draw_result(result, pipeline.osd_img)

        target = select_best_target(result)

        if target is not None:
            display_box = target[0:4]
            x1, y1, x2, y2, confidence = display_box_to_vision_bounds(target)
            vision_frame = None

            try:
                os.exitpoint()
                vision_frame = pipeline.capture_vision_frame()
                center_x, center_y, center_source, geometry_score = locate_target_center(
                    vision_frame,
                    (x1, y1, x2, y2),
                    previous_center
                )
            finally:
                vision_frame = None

            previous_center = (center_x, center_y)

            error_x = center_x - image_center_x
            error_y = center_y - image_center_y
            error_x, error_y = apply_dead_zone(error_x, error_y)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                message = "T,{},{},{}\r\n".format(
                    error_x,
                    error_y,
                    int(confidence * 100)
                )
                uart.write(message)
                last_send_ms = now_ms

                if DEBUG_PRINT:
                    print(
                        "TARGET",
                        "source=", center_source,
                        "display_box=", display_box,
                        "vision_box=", (x1, y1, x2, y2),
                        "center=", (center_x, center_y),
                        "error=", (error_x, error_y),
                        "geometry=", geometry_score,
                        "confidence=", int(confidence * 100)
                    )
        else:
            previous_center = None

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                uart.write("N\r\n")
                last_send_ms = now_ms

                if DEBUG_PRINT:
                    print("NO TARGET")

        # 清空MSPM0回传，避免ACK数据在接收缓冲区累计。
        received = uart.read()
        if DEBUG_PRINT and received:
            print("UART RX:", received)
        pipeline.show_osd()

        ai_frame = None
        ai_array = None
        gc_counter += 1

        if gc_counter >= 8:
            gc_counter = 0
            gc.collect()

except KeyboardInterrupt:
    print("K230程序已由用户停止")

except BaseException as error:
    print("K230运行异常:")
    sys.print_exception(error)

finally:
    if yolo is not None:
        try:
            yolo.deinit()
        except BaseException as error:
            print("YOLO清理异常:")
            sys.print_exception(error)

    if pipeline is not None:
        try:
            pipeline.destroy()
        except BaseException as error:
            print("视觉管线清理异常:")
            sys.print_exception(error)

    if uart is not None:
        try:
            uart.deinit()
        except BaseException as error:
            print("UART清理异常:")
            sys.print_exception(error)

    gc.collect()
    print("K230资源释放完成")
