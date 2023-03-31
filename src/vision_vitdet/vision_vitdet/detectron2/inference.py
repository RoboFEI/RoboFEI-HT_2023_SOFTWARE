#!/usr/bin/env python3.9
import numpy as np
import torch
import cv2

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import LazyConfig, instantiate
from detectron2.data.detection_utils import read_image
from detectron2.data import MetadataCatalog
from detectron2.utils.visualizer import ColorMode
from detectron2.engine.defaults import create_ddp_model
from detectron2.data.detection_utils import check_image_size
from detectron2.data.datasets import register_coco_instances
import detectron2
import copy
import glob

img_path = "/home/jonas/RoboFEI_2022/detectron2/bola.jpg"

cfg = LazyConfig.load("/home/jonas/RoboFEI_2022/detectron2/projects/ViTDet/configs/COCO/mask_rcnn_vitdet_b_100ep.py")

# edit the config to utilize common Batch Norm
cfg.model.backbone.norm = "BN"
cfg.model.roi_heads.num_classes = 1


model = instantiate(cfg.model)
model.to(cfg.train.device)
model = create_ddp_model(model)

DetectionCheckpointer(model).load("/home/jonas/RoboFEI_2022/detectron2/train_ball/model_final.pth")  # load a file, usually from cfg.MODEL.WEIGHTS
register_coco_instances("ball_train", {},"/home/jonas/RoboFEI_2022/detectron2/train/_annotations.coco.json", "/home/jonas/RoboFEI_2022/detectron2/train")
MetadataCatalog.get("ball_train").thing_classes = ['ball']
MetadataCatalog.get("ball_test").thing_classes = ['ball']


# read image for inference input
# use PIL, to be consistent with evaluation
src = cv2.imread(img_path)
img = cv2.resize(src, (1024, 1024))
img = torch.from_numpy(np.ascontiguousarray(img))
img = img.permute(2, 0, 1)  # HWC -> CHW
print(img.size())

if torch.cuda.is_available():
    img = img.cuda()
inputs = [{"image": img}]

# run the model
model.eval()

def mapper(dataset_dict):
    dataset_dict = copy.deepcopy(dataset_dict)
    dicts = []
    for i in range(len(dataset_dict)):
        image = read_image(dataset_dict[i]["file_name"], format="BGR")
        temp = image.copy()
        check_image_size(dataset_dict[i], temp)
        image = torch.from_numpy(temp)
        dicts.append({
           "image": image,
           'height': 720,
           'width': 1280,
        })
    return dicts

dataset_dict = detectron2.data.get_detection_dataset_dicts('ball_train')
data= mapper(dataset_dict)

with torch.no_grad():
  outputs = model(data)
