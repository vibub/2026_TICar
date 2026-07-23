import gc
import os
import time

import image
import nncase_runtime as nn
from media.display import *
from media.media import *
from media.sensor import *


class VisionPipeline:
    """为显示、传统视觉和 YOLO 提供独立分辨率的摄像头通道。"""

    def __init__(
        self,
        cv_size=(320, 240),
        ai_size=(640, 360),
        display_size=(800, 480),
        hmirror=True,
        vflip=True,
    ):
        # 传统视觉使用低分辨率 RGB565，保证红线检测可以稳定高频运行。
        self.cv_size = (
            ALIGN_UP(cv_size[0], 16),
            cv_size[1]
        )
        # YOLO 使用独立的 RGBP888 通道，避免传统视觉降分辨率影响模型输入。
        self.ai_size = (
            ALIGN_UP(ai_size[0], 16),
            ai_size[1]
        )
        self.display_size = (
            ALIGN_UP(display_size[0], 16),
            display_size[1]
        )
        # CanMV-K230-LP4 V3.0 的摄像头安装方向相对旧板旋转了 180°。
        # 同时启用水平镜像和垂直翻转，可让显示、传统视觉和 AI 三个通道方向一致。
        self.hmirror = bool(hmirror)
        self.vflip = bool(vflip)

        self.sensor = None
        self.osd_img = None
        self._sensor_started = False
        self._display_initialized = False
        self._media_initialized = False

    def create(self, sensor=None):
        os.exitpoint(os.EXITPOINT_ENABLE)
        print("S20.1: 收缩AI内存池")
        nn.shrink_memory_pool()

        print("S20.2: 复位Sensor")
        self.sensor = Sensor() if sensor is None else sensor
        self.sensor.reset()

        # Sensor 翻转配置为全局设置，必须在启动摄像头前完成。
        # 旋转 180° 等价于同时进行水平镜像和垂直翻转。
        self.sensor.set_hmirror(self.hmirror)
        self.sensor.set_vflip(self.vflip)
        print(
            "S20.3: Sensor复位完成，hmirror={} vflip={}".format(
                self.hmirror,
                self.vflip,
            )
        )

        # 通道0直接绑定LCD视频层。
        self.sensor.set_framesize(
            width=self.display_size[0],
            height=self.display_size[1],
            chn=CAM_CHN_ID_0
        )
        self.sensor.set_pixformat(
            Sensor.YUV420SP,
            chn=CAM_CHN_ID_0
        )

        # 通道1输出低分辨率 RGB565，供红线、路口及其他传统视觉算法使用。
        self.sensor.set_framesize(
            width=self.cv_size[0],
            height=self.cv_size[1],
            chn=CAM_CHN_ID_1
        )
        self.sensor.set_pixformat(
            Sensor.RGB565,
            chn=CAM_CHN_ID_1
        )

        # 通道2保持较高分辨率 RGBP888，供 YOLO 推理使用。
        self.sensor.set_framesize(
            width=self.ai_size[0],
            height=self.ai_size[1],
            chn=CAM_CHN_ID_2
        )
        self.sensor.set_pixformat(
            Sensor.RGBP888,
            chn=CAM_CHN_ID_2
        )

        print(
            "S20.3A: 通道配置 display={} cv={} ai={}".format(
                self.display_size,
                self.cv_size,
                self.ai_size,
            )
        )

        self.osd_img = image.Image(
            self.display_size[0],
            self.display_size[1],
            image.ARGB8888
        )

        bind_info = self.sensor.bind_info(
            x=0,
            y=0,
            chn=CAM_CHN_ID_0
        )
        Display.bind_layer(
            **bind_info,
            layer=Display.LAYER_VIDEO1
        )

        print("S20.4: 初始化Display")
        Display.init(
            Display.ST7701,
            width=self.display_size[0],
            height=self.display_size[1],
            to_ide=True
        )
        self._display_initialized = True
        print("S20.5: Display初始化完成")

        print("S20.6: 初始化MediaManager")
        MediaManager.init()
        self._media_initialized = True
        print("S20.7: MediaManager初始化完成")

        print("S20.8: 启动Sensor")
        self.sensor.run()
        self._sensor_started = True
        print("S20.9: Sensor启动完成")

    def capture_ai_frame(self):
        frame = self.sensor.snapshot(chn=CAM_CHN_ID_2)
        return frame, frame.to_numpy_ref()

    def capture_vision_frame(self):
        return self.sensor.snapshot(chn=CAM_CHN_ID_1)

    def clear_osd(self):
        if self.osd_img is not None:
            self.osd_img.clear()

    def show_osd(self):
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3
        )

    def destroy(self):
        if self._sensor_started and self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
            self._sensor_started = False

        if self._display_initialized:
            try:
                Display.deinit()
            except Exception:
                pass
            self._display_initialized = False

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)

        if self._media_initialized:
            try:
                MediaManager.deinit()
            except Exception:
                pass
            self._media_initialized = False

        self.osd_img = None
        self.sensor = None
        gc.collect()
