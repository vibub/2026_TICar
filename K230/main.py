import gc
import os
import sys
import time


K230_UART_TX_IO = 44
K230_UART_RX_IO = 45

# DELIVERY 为比赛视觉主程序；LINE 和 TARGET 保留独立联调入口。
RUN_MODE = "DELIVERY"
LINE_MODE = "LINE"
DELIVERY_MODE = "DELIVERY"
TARGET_MODE = "TARGET"

LINE_CV_SIZE = (320, 240)
TARGET_CV_SIZE = (640, 360)
AI_SIZE = (640, 360)
DISPLAY_SIZE = (800, 480)

TARGET_MODEL_INPUT_SIZE = [320, 320]
TARGET_MODEL_PATH = "/sdcard/models/yolo11n_det_320.kmodel"
TARGET_LABELS = {0: "Target"}
TARGET_CONFIDENCE_THRESHOLD = 0.8

# 初版沿用 digit_camera.py 已验证的 320×320、1～8 类模型配置。
DIGIT_MODEL_INPUT_SIZE = [320, 320]
DIGIT_MODEL_PATH = "/sdcard/models/yolo11n_det_320.kmodel"
DIGIT_LABELS = ["1", "2", "3", "4", "5", "6", "7", "8"]
DIGIT_CONFIDENCE_THRESHOLD = 0.60
DIGIT_INFERENCE_INTERVAL_MS = 200
DIGIT_DEBUG_INTERVAL_RUNS = 10
DIGIT_OVERLAY_HOLD_MS = 800
OSD_UPDATE_INTERVAL_MS = 80

NMS_THRESHOLD = 0.45
DEAD_ZONE_X = 5
DEAD_ZONE_Y = 5
TARGET_SEND_INTERVAL_MS = 100
LINE_SEND_INTERVAL_MS = 40
LINE_DEBUG_INTERVAL_FRAMES = 25
GC_INTERVAL_FRAMES = 32
UART_RX_LINE_SIZE = 64
DEBUG_PRINT = True


fpioa = None
uart = None
pipeline = None
yolo = None
uart_rx_line = bytearray()
uart_rx_discard = False
uart_rx_byte_count = 0
uart_rx_line_count = 0
uart_valid_command_count = 0
uart_last_line = "NONE"
uart_last_status = "WAIT V"
uart_last_ack_ms = 0
osd_error_logged = False


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
    fpioa.set_function(K230_UART_TX_IO, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_IO, FPIOA.UART2_RXD)

    result = UART(
        UART.UART2,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )
    print("S11: UART初始化完成")
    return result


def poll_uart_lines():
    """非阻塞读取 CRLF 文本行，超长行丢弃到下一个换行符。"""
    global uart_rx_line, uart_rx_discard
    global uart_rx_byte_count, uart_rx_line_count, uart_last_line

    completed = []
    received = uart.read()
    if not received:
        return completed

    uart_rx_byte_count += len(received)
    for byte in received:
        if uart_rx_discard:
            if byte == 10:
                uart_rx_discard = False
                uart_rx_line = bytearray()
            continue

        if byte == 13:
            continue
        if byte == 10:
            if len(uart_rx_line) != 0:
                try:
                    decoded_line = uart_rx_line.decode()
                    completed.append(decoded_line)
                    uart_rx_line_count += 1
                    uart_last_line = decoded_line
                except Exception:
                    uart_last_line = "DECODE ERR"
                uart_rx_line = bytearray()
            continue

        if len(uart_rx_line) < UART_RX_LINE_SIZE - 1:
            uart_rx_line.append(byte)
        else:
            uart_rx_line = bytearray()
            uart_rx_discard = True

    return completed


def process_visual_commands(
    current_command,
    pharmacy_consensus,
    route_consensus,
    parse_visual_command,
):
    """处理 MSPM0 V 命令；旧 ACK/调试回包只消费，不形成应答回环。"""
    global uart_valid_command_count, uart_last_status, uart_last_ack_ms

    for line in poll_uart_lines():
        command = parse_visual_command(line)
        if command is None:
            if not line.startswith("ACK,"):
                uart_last_status = "INVALID LINE"
                if DEBUG_PRINT:
                    print("UART RX invalid:", line)
            continue

        epoch_changed = current_command.epoch != command.epoch
        route_changed = (
            current_command.mode != command.mode or
            current_command.route_region != command.route_region or
            current_command.target_digit != command.target_digit
        )
        if epoch_changed:
            pharmacy_consensus.reset(command.epoch)
            route_consensus.reset((command.epoch << 4) | command.route_region)
        elif route_changed:
            route_consensus.reset((command.epoch << 4) | command.route_region)

        current_command = command
        uart.write(
            "A,V,{},{},{},{}\r\n".format(
                command.mode,
                command.target_digit,
                command.route_region,
                command.epoch,
            )
        )
        uart_valid_command_count += 1
        uart_last_status = "V OK"
        uart_last_ack_ms = time.ticks_ms()
        if DEBUG_PRINT:
            print(
                "VISUAL COMMAND",
                "mode=", command.mode,
                "target=", command.target_digit,
                "region=", command.route_region,
                "epoch=", command.epoch,
            )
    return current_command


def format_line_message(result):
    if result.valid:
        return "L,1,{},{},{},{}\r\n".format(
            result.error_x,
            result.angle_d10,
            result.quality,
            result.direction_mask,
        )
    # 无线帧仍证明视觉循环在线，但禁止下位机使用旧误差继续控制。
    return "L,0,0,0,{},0\r\n".format(result.quality)


def _scale_coordinate(value, source_size, display_size):
    return int(int(value) * int(display_size) // int(source_size))


def _draw_osd_text(osd, x, y, text, color):
    """统一使用高级文字接口，避免不同固件的基础字体缩放差异。"""
    osd.draw_string_advanced(x, y, 18, str(text), color=color)


def draw_delivery_overlay(
    line_result,
    command,
    last_detection,
    last_detection_ms,
    now_ms,
    max_line_send_gap_ms,
    max_digit_inference_ms,
):
    """把红线、数字和 UART 状态统一重画到 800×480 OSD。"""
    global osd_error_logged

    try:
        osd = pipeline.osd_img
        pipeline.clear_osd()

        white = (255, 255, 255)
        green = (0, 255, 0)
        red = (255, 0, 0)
        yellow = (255, 255, 0)
        cyan = (0, 255, 255)

        # 红线参考中心和当前控制点使用 320×240 到显示坐标的缩放。
        display_center_x = DISPLAY_SIZE[0] // 2
        control_y = _scale_coordinate(204, LINE_CV_SIZE[1], DISPLAY_SIZE[1])
        line_x = _scale_coordinate(
            (LINE_CV_SIZE[0] // 2) + line_result.error_x,
            LINE_CV_SIZE[0],
            DISPLAY_SIZE[0],
        )
        line_color = green if line_result.valid else red
        osd.draw_line(
            display_center_x, DISPLAY_SIZE[1] // 2,
            display_center_x, DISPLAY_SIZE[1] - 1,
            color=cyan, thickness=2,
        )
        osd.draw_line(
            line_x - 12, control_y, line_x + 12, control_y,
            color=line_color, thickness=3,
        )
        osd.draw_line(
            line_x, control_y - 12, line_x, control_y + 12,
            color=line_color, thickness=3,
        )

        _draw_osd_text(
            osd, 8, 6,
            "LINE V{} E{} A{} Q{} M{} P{}".format(
                int(line_result.valid),
                line_result.error_x,
                line_result.angle_d10,
                line_result.quality,
                line_result.direction_mask,
                line_result.point_count,
            ),
            line_color,
        )
        _draw_osd_text(
            osd, 8, 30,
            "V MODE{} TARGET{} REGION{} EPOCH{}".format(
                command.mode,
                command.target_digit,
                command.route_region,
                command.epoch,
            ),
            yellow,
        )
        _draw_osd_text(
            osd, 8, 54,
            "UART B{} L{} OK{} {}".format(
                uart_rx_byte_count,
                uart_rx_line_count,
                uart_valid_command_count,
                uart_last_status,
            ),
            green if uart_valid_command_count else yellow,
        )
        _draw_osd_text(
            osd, 8, 78,
            "RX " + uart_last_line[:28],
            white,
        )
        _draw_osd_text(
            osd, 8, 102,
            "GAP{}ms YOLO{}ms".format(
                max_line_send_gap_ms,
                max_digit_inference_ms,
            ),
            white,
        )

        # 数字推理只有约 5 Hz，缓存框在 TTL 内每次 OSD 刷新时重新绘制。
        if last_detection is not None:
            detection_age_ms = time.ticks_diff(now_ms, last_detection_ms)
            if 0 <= detection_age_ms <= DIGIT_OVERLAY_HOLD_MS:
                draw_x = _scale_coordinate(
                    last_detection.x, AI_SIZE[0], DISPLAY_SIZE[0])
                draw_y = _scale_coordinate(
                    last_detection.y, AI_SIZE[1], DISPLAY_SIZE[1])
                draw_w = max(1, _scale_coordinate(
                    last_detection.width, AI_SIZE[0], DISPLAY_SIZE[0]))
                draw_h = max(1, _scale_coordinate(
                    last_detection.height, AI_SIZE[1], DISPLAY_SIZE[1]))
                osd.draw_rectangle(
                    draw_x, draw_y, draw_w, draw_h,
                    color=yellow, thickness=3,
                )
                label_y = max(126, draw_y - 24)
                _draw_osd_text(
                    osd, draw_x, label_y,
                    "D{} S{} C{}".format(
                        last_detection.digit,
                        last_detection.side,
                        int(last_detection.confidence * 100),
                    ),
                    yellow,
                )

        pipeline.show_osd()
        osd_error_logged = False
    except BaseException as error:
        if DEBUG_PRINT and not osd_error_logged:
            osd_error_logged = True
            print("OSD绘制异常:")
            sys.print_exception(error)


def create_digit_yolo(YOLO11):
    print("S30D: 加载数字YOLO模型", DIGIT_MODEL_PATH)
    print("S30D: 输入", DIGIT_MODEL_INPUT_SIZE, "标签", DIGIT_LABELS)
    detector = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=DIGIT_MODEL_PATH,
        labels=DIGIT_LABELS,
        rgb888p_size=list(AI_SIZE),
        model_input_size=DIGIT_MODEL_INPUT_SIZE,
        display_size=list(pipeline.display_size),
        conf_thresh=DIGIT_CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=50,
        debug_mode=0,
    )
    detector.config_preprocess()
    print("S31D: 数字YOLO模型加载完成")
    return detector


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

    while True:
        os.exitpoint()
        clock.tick()

        frame = pipeline.capture_vision_frame()
        result = detector.detect(frame)
        frame = None

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_send_ms) >= LINE_SEND_INTERVAL_MS:
            uart.write(format_line_message(result))
            last_send_ms = now_ms
            received_lines = poll_uart_lines()
            if DEBUG_PRINT and received_lines:
                print("UART RX:", received_lines)

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


def run_delivery_mode(
    RedLineDetector,
    VisionPipeline,
    YOLO11,
    DigitConsensus,
    VisualCommand,
    make_detections,
    parse_visual_command,
    format_digit_frame,
    is_digit_inference_due,
    constants,
):
    """比赛视觉循环：红线优先，按 MSPM0 状态低频运行数字 YOLO。"""
    global pipeline, yolo

    pipeline = VisionPipeline(
        cv_size=LINE_CV_SIZE,
        ai_size=AI_SIZE,
        display_size=DISPLAY_SIZE,
    )
    pipeline.create()
    red_line_detector = RedLineDetector()
    pharmacy_consensus = DigitConsensus(history_size=5, required_votes=3)
    route_consensus = DigitConsensus(history_size=5, required_votes=3)
    command = VisualCommand(constants["VISUAL_MODE_OFF"], 0, 0, 0)

    frame_count = 0
    digit_run_count = 0
    first_result_logged = False
    last_line_send_ms = time.ticks_ms()
    last_digit_run_ms = time.ticks_ms()
    last_osd_ms = time.ticks_ms()
    last_detection = None
    last_detection_ms = 0
    max_line_send_gap_ms = 0
    max_digit_inference_ms = 0
    clock = time.clock()

    print("K230比赛视觉启动：红线每帧，数字按V命令运行")
    print("V命令必须从UART1 RX IO{}输入，ACK从TX IO{}输出".format(
        K230_UART_RX_IO, K230_UART_TX_IO))
    print("CanMV IDE USB终端输入不会进入该硬件UART；命令必须以真实LF结尾")

    while True:
        os.exitpoint()
        clock.tick()

        previous_mode = command.mode
        previous_epoch = command.epoch
        command = process_visual_commands(
            command,
            pharmacy_consensus,
            route_consensus,
            parse_visual_command,
        )
        if (command.mode == constants["VISUAL_MODE_OFF"] or
                command.epoch != previous_epoch or
                command.mode != previous_mode):
            last_detection = None
            last_detection_ms = 0

        # 红线始终优先运行和发送，数字推理不能跳过本轮传统视觉更新。
        vision_frame = pipeline.capture_vision_frame()
        line_result = red_line_detector.detect(vision_frame)
        vision_frame = None

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_line_send_ms) >= LINE_SEND_INTERVAL_MS:
            line_gap_ms = time.ticks_diff(now_ms, last_line_send_ms)
            if line_gap_ms > max_line_send_gap_ms:
                max_line_send_gap_ms = line_gap_ms
            uart.write(format_line_message(line_result))
            last_line_send_ms = now_ms

        digit_enabled = command.mode != constants["VISUAL_MODE_OFF"]
        if digit_enabled and is_digit_inference_due(
            now_ms,
            last_digit_run_ms,
            DIGIT_INFERENCE_INTERVAL_MS,
        ):
            if yolo is None:
                yolo = create_digit_yolo(YOLO11)

            ai_frame = None
            ai_array = None
            inference_start_ms = time.ticks_ms()
            try:
                ai_frame, ai_array = pipeline.capture_ai_frame()
                yolo_result = yolo.run(ai_array)

                if DEBUG_PRINT and not first_result_logged:
                    first_result_logged = True
                    print(
                        "DIGIT RESULT STRUCTURE",
                        "parts=", len(yolo_result),
                        "boxes=", len(yolo_result[0]),
                        "classes=", len(yolo_result[1]),
                        "scores=", len(yolo_result[2]),
                    )

                target_filter = (
                    command.target_digit
                    if command.mode == constants["VISUAL_MODE_TARGET"]
                    else None
                )
                detections = make_detections(
                    yolo_result,
                    DIGIT_LABELS,
                    DIGIT_CONFIDENCE_THRESHOLD,
                    target_filter,
                )
                detection = detections[0] if detections else None
                flags = 0
                consensus = (
                    route_consensus
                    if command.mode == constants["VISUAL_MODE_TARGET"]
                    else pharmacy_consensus
                )

                if detection is not None:
                    last_detection = detection
                    last_detection_ms = now_ms
                    flags |= constants["DIGIT_FLAG_VALID"]
                    if target_filter in (None, 0) or detection.digit == target_filter:
                        flags |= constants["DIGIT_FLAG_TARGET_MATCH"]
                    status = consensus.observe(detection, now_ms)
                    if status["confirmed"]:
                        flags |= constants["DIGIT_FLAG_CONSENSUS"]
                    if pharmacy_consensus.status()["confirmed"]:
                        flags |= constants["DIGIT_FLAG_LOCKED"]

                    uart.write(
                        format_digit_frame(
                            True,
                            detection.digit,
                            detection.x,
                            detection.y,
                            detection.width,
                            detection.height,
                            detection.side,
                            int(detection.confidence * 100),
                            flags,
                        )
                    )
                else:
                    if pharmacy_consensus.status()["confirmed"]:
                        flags |= constants["DIGIT_FLAG_LOCKED"]
                    uart.write(format_digit_frame(False, 0, 0, 0, 0, 0, 0, 0, flags))

                # DELIVERY 模式由统一 OSD 使用缓存框重画，避免低频 YOLO 框闪烁。
            except BaseException as error:
                uart.write(
                    format_digit_frame(
                        False, 0, 0, 0, 0, 0, 0, 0,
                        constants["DIGIT_FLAG_ERROR"],
                    )
                )
                print("数字识别异常:")
                sys.print_exception(error)
            finally:
                ai_frame = None
                ai_array = None

            inference_ms = time.ticks_diff(time.ticks_ms(), inference_start_ms)
            if inference_ms > max_digit_inference_ms:
                max_digit_inference_ms = inference_ms
            last_digit_run_ms = time.ticks_ms()
            digit_run_count += 1

            if DEBUG_PRINT and digit_run_count % DIGIT_DEBUG_INTERVAL_RUNS == 0:
                print(
                    "DIGIT PERF",
                    "last_ms=", inference_ms,
                    "max_ms=", max_digit_inference_ms,
                    "line_max_gap_ms=", max_line_send_gap_ms,
                )

        osd_now_ms = time.ticks_ms()
        if time.ticks_diff(osd_now_ms, last_osd_ms) >= OSD_UPDATE_INTERVAL_MS:
            draw_delivery_overlay(
                line_result,
                command,
                last_detection,
                last_detection_ms,
                osd_now_ms,
                max_line_send_gap_ms,
                max_digit_inference_ms,
            )
            last_osd_ms = osd_now_ms

        frame_count += 1
        if DEBUG_PRINT and frame_count % LINE_DEBUG_INTERVAL_FRAMES == 0:
            print(
                "DELIVERY LINE",
                "valid=", int(line_result.valid),
                "error=", line_result.error_x,
                "mask=", line_result.direction_mask,
                "visual_mode=", command.mode,
                "target=", command.target_digit,
                "region=", command.route_region,
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
        kmodel_path=TARGET_MODEL_PATH,
        labels=TARGET_LABELS,
        rgb888p_size=list(AI_SIZE),
        model_input_size=TARGET_MODEL_INPUT_SIZE,
        display_size=list(pipeline.display_size),
        conf_thresh=TARGET_CONFIDENCE_THRESHOLD,
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

        poll_uart_lines()
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
    elif RUN_MODE == DELIVERY_MODE:
        from red_line import RedLineDetector
        from libs.YOLO import YOLO11
        from digit_logic import (
            DIGIT_FLAG_CONSENSUS,
            DIGIT_FLAG_ERROR,
            DIGIT_FLAG_LOCKED,
            DIGIT_FLAG_TARGET_MATCH,
            DIGIT_FLAG_VALID,
            VISUAL_MODE_OFF,
            VISUAL_MODE_TARGET,
            DigitConsensus,
            VisualCommand,
            format_digit_frame,
            is_digit_inference_due,
            make_detections,
            parse_visual_command,
        )
    elif RUN_MODE == TARGET_MODE:
        from libs.YOLO import YOLO11
        from target_geometry import locate_target_center
    else:
        raise ValueError("未知RUN_MODE: " + str(RUN_MODE))
    print("S01: 模块导入完成")

    uart = create_uart(UART, FPIOA)

    if RUN_MODE == LINE_MODE:
        run_line_mode(RedLineDetector, VisionPipeline)
    elif RUN_MODE == DELIVERY_MODE:
        run_delivery_mode(
            RedLineDetector,
            VisionPipeline,
            YOLO11,
            DigitConsensus,
            VisualCommand,
            make_detections,
            parse_visual_command,
            format_digit_frame,
            is_digit_inference_due,
            {
                "VISUAL_MODE_OFF": VISUAL_MODE_OFF,
                "VISUAL_MODE_TARGET": VISUAL_MODE_TARGET,
                "DIGIT_FLAG_VALID": DIGIT_FLAG_VALID,
                "DIGIT_FLAG_TARGET_MATCH": DIGIT_FLAG_TARGET_MATCH,
                "DIGIT_FLAG_CONSENSUS": DIGIT_FLAG_CONSENSUS,
                "DIGIT_FLAG_LOCKED": DIGIT_FLAG_LOCKED,
                "DIGIT_FLAG_ERROR": DIGIT_FLAG_ERROR,
            },
        )
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
