import gc
import os
import time

import image
import nncase_runtime as nn
from media.display import *
from media.media import *
from media.sensor import *


class VisionPipeline:
    """为显示、传统视觉和 YOLO 提供独立的摄像头通道。"""

    def __init__(self, vision_size=(640, 360), display_size=(800, 480)):
        self.vision_size = (
            ALIGN_UP(vision_size[0], 16),
            vision_size[1]
        )
        self.display_size = (
            ALIGN_UP(display_size[0], 16),
            display_size[1]
        )

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
        print("S20.3: Sensor复位完成")

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

        # 通道1直接输出RGB565，供矩形和圆检测使用。
        self.sensor.set_framesize(
            width=self.vision_size[0],
            height=self.vision_size[1],
            chn=CAM_CHN_ID_1
        )
        self.sensor.set_pixformat(
            Sensor.RGB565,
            chn=CAM_CHN_ID_1
        )

        # 通道2输出RGBP888，供YOLO推理使用。
        self.sensor.set_framesize(
            width=self.vision_size[0],
            height=self.vision_size[1],
            chn=CAM_CHN_ID_2
        )
        self.sensor.set_pixformat(
            Sensor.RGBP888,
            chn=CAM_CHN_ID_2
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
