"""CanMV K230 摄像头实时预览测试。

运行后，摄像头画面会显示在 CanMV IDE 的帧缓冲窗口中。
"""

import gc
import os
import time

from media.display import *
from media.media import *
from media.sensor import *


# 先使用较低分辨率验证摄像头和 IDE 通信，避免 FHD JPEG 预览负载过高。
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480
PREVIEW_FPS = 30


def camera_preview():
    sensor = None
    sensor_started = False
    display_initialized = False
    media_initialized = False

    # 让 CanMV IDE 的“停止”按钮能够中断下面的循环。
    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        print("正在初始化摄像头...")
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(
            width=PREVIEW_WIDTH,
            height=PREVIEW_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        # VIRT 只向 CanMV IDE 输出，不需要外接 LCD 或 HDMI 屏幕。
        Display.init(
            Display.VIRT,
            width=PREVIEW_WIDTH,
            height=PREVIEW_HEIGHT,
            fps=PREVIEW_FPS,
            to_ide=True,
        )
        display_initialized = True

        MediaManager.init()
        media_initialized = True

        sensor.run()
        sensor_started = True
        print("摄像头已启动，画面应出现在 CanMV IDE 帧缓冲窗口中。")

        clock = time.clock()
        while True:
            os.exitpoint()
            clock.tick()

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            Display.show_image(img)
            print("FPS: %.2f" % clock.fps())

    except KeyboardInterrupt:
        print("用户停止摄像头预览。")
    except BaseException as error:
        print("摄像头预览异常:", error)
        raise
    finally:
        if sensor_started:
            sensor.stop()

        if display_initialized:
            Display.deinit()

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)

        if media_initialized:
            MediaManager.deinit()

        sensor = None
        gc.collect()
        print("摄像头资源已释放。")


if __name__ == "__main__":
    camera_preview()
