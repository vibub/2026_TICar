"""
实验名称：在线训练-YOLO图像检测: 基于图片
实验平台：01Studio CanMV K230
01科技（01Studio）在线训练平台：https://ai.01studio.cc
"""

from libs.YOLO import YOLO11
from libs.Utils import *
import os, sys, gc
import ulab.numpy as np
import image

# 这里为自动生成内容，自定义场景请修改为您自己的测试图片、模型路径、标签名称、模型输入大小
img_path="/sdcard/val.jpg"
kmodel_path="/sdcard/yolo11n_det_640.kmodel"
labels = {0: '1', 1: '2', 2: '3', 3: '4', 4: '5', 5: '6', 6: '7', 7: '8', 8: '9'}
model_input_size = [640, 640]

img, img_ori = read_image(img_path)
rgb888p_size = [img.shape[2], img.shape[1]]

# 初始化YOLO11实例
confidence_threshold = 0.6  # 置信度
nms_threshold = 0.45
yolo = YOLO11(
    task_type="detect",
    mode="image",
    kmodel_path=kmodel_path,
    labels=labels,
    rgb888p_size=rgb888p_size,
    model_input_size=model_input_size,
    conf_thresh=confidence_threshold,
    nms_thresh=nms_threshold,
    max_boxes_num=50,
    debug_mode=0,
)
yolo.config_preprocess()

res = yolo.run(img)  # 推理图像
yolo.draw_result(res, img_ori)  # 绘制结果

# 释放资源
yolo.deinit()
gc.collect()
