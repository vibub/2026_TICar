"""使用比赛主视觉管线对数字 1～8 进行摄像头实时检测。"""

import gc
import os
import sys
import time

from libs.YOLO import YOLO11
from digit_logic import make_detections
from vision_pipeline import VisionPipeline


KMODEL_PATH = "/sdcard/models/yolo11n_det_320.kmodel"
LABELS = ["1", "2", "3", "4", "5", "6", "7", "8"]
MODEL_INPUT_SIZE = [320, 320]

RGB888P_SIZE = [640, 360]
DISPLAY_SIZE = [800, 480]
CONFIDENCE_THRESHOLD = 0.60
NMS_THRESHOLD = 0.45


def run_digit_detector():
    """独立验证数字模型、检测框和新车体左右方位。"""
    pipeline = None
    detector = None

    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        print("初始化比赛摄像头管线...")
        pipeline = VisionPipeline(
            cv_size=(320, 240),
            ai_size=tuple(RGB888P_SIZE),
            display_size=tuple(DISPLAY_SIZE),
        )
        pipeline.create()

        print("加载数字检测模型:", KMODEL_PATH)
        detector = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=list(pipeline.display_size),
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=50,
            debug_mode=0,
        )
        detector.config_preprocess()

        print("数字检测已启动，可识别:", LABELS)
        clock = time.clock()
        frame_count = 0

        while True:
            os.exitpoint()
            clock.tick()

            ai_frame, ai_array = pipeline.capture_ai_frame()
            result = detector.run(ai_array)
            detections = make_detections(
                result,
                LABELS,
                CONFIDENCE_THRESHOLD,
            )

            pipeline.clear_osd()
            detector.draw_result(result, pipeline.osd_img)
            pipeline.show_osd()

            frame_count += 1
            if frame_count >= 30:
                if detections:
                    best = detections[0]
                    print(
                        "DIGIT",
                        "number=", best.digit,
                        "box=", (best.x, best.y, best.width, best.height),
                        "side=", best.side,
                        "confidence=", int(best.confidence * 100),
                        "fps=", clock.fps(),
                    )
                else:
                    print("DIGIT none, fps=", clock.fps())
                frame_count = 0
                gc.collect()

            ai_frame = None
            ai_array = None
            result = None

    except KeyboardInterrupt:
        print("用户停止数字检测。")
    except BaseException as error:
        print("数字检测异常:")
        sys.print_exception(error)
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as error:
                print("YOLO清理异常:")
                sys.print_exception(error)

        if pipeline is not None:
            try:
                pipeline.destroy()
            except BaseException as error:
                print("摄像头管线清理异常:")
                sys.print_exception(error)

        gc.collect()
        print("数字检测资源已释放。")


if __name__ == "__main__":
    run_digit_detector()
