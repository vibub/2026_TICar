import gc
import os
import sys
import time


K230_UART_TX_IO = 3
K230_UART_RX_IO = 4

# LINE 为比赛红线巡线；TARGET 保留原有 YOLO 靶标和云台联调功能。
RUN_MODE = "LINE"
LINE_MODE = "LINE"
TARGET_MODE = "TARGET"

LINE_CV_SIZE = (320, 240)
TARGET_CV_SIZE = (640, 360)
AI_SIZE = (640, 360)
DISPLAY_SIZE = (800, 480)
MODEL_INPUT_SIZE = [320, 320]
MODEL_PATH = "/sdcard/models/yolo11n_det_320.kmodel"
TARGET_LABELS = {0: "Target"}

CONFIDENCE_THRESHOLD = 0.8
NMS_THRESHOLD = 0.45
DEAD_ZONE_X = 5
DEAD_ZONE_Y = 5
TARGET_SEND_INTERVAL_MS = 100
LINE_SEND_INTERVAL_MS = 40
LINE_DEBUG_INTERVAL_FRAMES = 25
GC_INTERVAL_FRAMES = 32
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

    # YOLO11 video 模式返回 AI 坐标系中的 [x, y, width, height]。
    return (
        int(box[0]),
        int(box[1]),
        int(box[2]),
        int(box[3]),
        best_score,
    )


def target_box_to_bounds(target):
    x, y, width, height, confidence = target
    x1 = max(0, min(AI_SIZE[0] - 1, int(x)))
    y1 = max(0, min(AI_SIZE[1] - 1, int(y)))
    x2 = max(x1 + 1, min(AI_SIZE[0], int(x + width)))
    y2 = max(y1 + 1, min(AI_SIZE[1], int(y + height)))
    return x1, y1, x2, y2, confidence


def apply_dead_zone(error_x, error_y):
    if abs(error_x) <= DEAD_ZONE_X:
        error_x = 0

    if abs(error_y) <= DEAD_ZONE_Y:
        error_y = 0

    return error_x, error_y


def create_uart(UART, FPIOA):
    global fpioa

    print("S10: 初始化UART")
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_IO, FPIOA.UART1_TXD)
    fpioa.set_function(K230_UART_RX_IO, FPIOA.UART1_RXD)

    result = UART(
        UART.UART1,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )
    print("S11: UART初始化完成")
    return result


def drain_uart_rx():
    """非阻塞清空 MSPM0 ACK，避免回包在 K230 接收缓冲区内累计。"""
    received = uart.read()
    if DEBUG_PRINT and received:
        print("UART RX:", received)


def run_line_mode(RedLineDetector, VisionPipeline):
    global pipeline

    print("S20: 初始化红线视觉管线")
    pipeline = VisionPipeline(
        cv_size=LINE_CV_SIZE,
        ai_size=AI_SIZE,
        display_size=DISPLAY_SIZE,
    )
    pipeline.create()
    detector = RedLineDetector()
    clock = time.clock()
    frame_count = 0
    last_send_ms = time.ticks_ms()

    if DEBUG_PRINT:
        print("K230红线视觉启动")
        print("传统视觉:", LINE_CV_SIZE, "AI:", AI_SIZE)
        print("UART1: 115200 8N1, TX IO", K230_UART_TX_IO,
              "RX IO", K230_UART_RX_IO)

    while True:
        os.exitpoint()
        clock.tick()

        frame = pipeline.capture_vision_frame()
        result = detector.detect(frame)
        frame = None

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_send_ms) >= LINE_SEND_INTERVAL_MS:
            if result.valid:
                message = "L,1,{},{},{},{}\r\n".format(
                    result.error_x,
                    result.angle_d10,
                    result.quality,
                    result.direction_mask,
                )
            else:
                # 无线帧仍证明视觉循环在线，但禁止下位机使用旧误差继续控制。
                message = "L,0,0,0,{},0\r\n".format(result.quality)

            uart.write(message)
            last_send_ms = now_ms
            drain_uart_rx()

        frame_count += 1
        if DEBUG_PRINT and frame_count % LINE_DEBUG_INTERVAL_FRAMES == 0:
            print(
                "LINE",
                "valid=", int(result.valid),
                "error=", result.error_x,
                "angle10=", result.angle_d10,
                "quality=", result.quality,
                "mask=", result.direction_mask,
                "points=", result.point_count,
                "fps=", clock.fps(),
            )

        if frame_count % GC_INTERVAL_FRAMES == 0:
            gc.collect()


def run_target_mode(YOLO11, VisionPipeline, locate_target_center):
    global pipeline, yolo

    print("S20: 初始化靶标视觉管线")
    # 旧靶标几何算法要求传统视觉帧和 YOLO 框使用相同的 640×360 坐标系。
    pipeline = VisionPipeline(
        cv_size=TARGET_CV_SIZE,
        ai_size=AI_SIZE,
        display_size=DISPLAY_SIZE,
    )
    pipeline.create()

    print("S30: 加载YOLO模型")
    yolo = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=MODEL_PATH,
        labels=TARGET_LABELS,
        rgb888p_size=list(AI_SIZE),
        model_input_size=MODEL_INPUT_SIZE,
        display_size=list(pipeline.display_size),
        conf_thresh=CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=50,
        debug_mode=0,
    )
    yolo.config_preprocess()
    print("S31: YOLO模型加载完成")

    image_center_x = AI_SIZE[0] // 2
    image_center_y = AI_SIZE[1] // 2
    last_send_ms = time.ticks_ms()
    previous_center = None
    gc_counter = 0

    while True:
        os.exitpoint()

        ai_frame, ai_array = pipeline.capture_ai_frame()
        result = yolo.run(ai_array)
        pipeline.clear_osd()
        yolo.draw_result(result, pipeline.osd_img)
        target = select_best_target(result)

        if target is not None:
            display_box = target[0:4]
            x1, y1, x2, y2, confidence = target_box_to_bounds(target)
            vision_frame = None

            try:
                vision_frame = pipeline.capture_vision_frame()
                center_x, center_y, center_source, geometry_score = locate_target_center(
                    vision_frame,
                    (x1, y1, x2, y2),
                    previous_center,
                )
            finally:
                vision_frame = None

            previous_center = (center_x, center_y)
            error_x = center_x - image_center_x
            error_y = center_y - image_center_y
            error_x, error_y = apply_dead_zone(error_x, error_y)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= TARGET_SEND_INTERVAL_MS:
                uart.write(
                    "T,{},{},{}\r\n".format(
                        error_x,
                        error_y,
                        int(confidence * 100),
                    )
                )
                last_send_ms = now_ms
                drain_uart_rx()

                if DEBUG_PRINT:
                    print(
                        "TARGET",
                        "source=", center_source,
                        "display_box=", display_box,
                        "center=", (center_x, center_y),
                        "error=", (error_x, error_y),
                        "geometry=", geometry_score,
                        "confidence=", int(confidence * 100),
                    )
        else:
            previous_center = None
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= TARGET_SEND_INTERVAL_MS:
                uart.write("N\r\n")
                last_send_ms = now_ms
                drain_uart_rx()

        pipeline.show_osd()
        ai_frame = None
        ai_array = None
        gc_counter += 1

        if gc_counter >= 8:
            gc_counter = 0
            gc.collect()


try:
    print("S00: 导入K230模块")
    from machine import UART, FPIOA
    from vision_pipeline import VisionPipeline

    if RUN_MODE == LINE_MODE:
        from red_line import RedLineDetector
    elif RUN_MODE == TARGET_MODE:
        from libs.YOLO import YOLO11
        from target_geometry import locate_target_center
    else:
        raise ValueError("未知RUN_MODE: " + str(RUN_MODE))
    print("S01: 模块导入完成")

    uart = create_uart(UART, FPIOA)

    if RUN_MODE == LINE_MODE:
        run_line_mode(RedLineDetector, VisionPipeline)
    else:
        run_target_mode(YOLO11, VisionPipeline, locate_target_center)

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
            print("摄像头管线清理异常:")
            sys.print_exception(error)

    if uart is not None:
        try:
            uart.deinit()
        except BaseException as error:
            print("UART清理异常:")
            sys.print_exception(error)

    gc.collect()
    print("K230资源释放完成")
