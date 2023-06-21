#!/usr/bin/env python3
import numpy as np
import torch
import cv2
from telnetlib import NOP

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import LazyConfig, instantiate
from detectron2.engine.defaults import create_ddp_model
from detectron2.data.detection_utils import read_image
from detectron2.data import MetadataCatalog
from detectron2.utils.visualizer import ColorMode
from detectron2.data.datasets import register_coco_instances
import glob
import time

img_path = "./bola.jpg"
threshold = 0.85

cfg = LazyConfig.load("./projects/ViTDet/configs/COCO/mask_rcnn_vitdet_b_100ep.py")

# edit the config to utilize common Batch Norm
cfg.model.backbone.norm = "BN"
cfg.model.roi_heads.num_classes = 2

model = instantiate(cfg.model)

# DetectionCheckpointer(model).load("./model_final_61ccd1.pkl")  # load a file, usually from cfg.MODEL.WEIGHTS
model.to(cfg.train.device)
model = create_ddp_model(model)
DetectionCheckpointer(model).load("./model_final_mask.pth")  # load a file, usually from cfg.MODEL.WEIGHTS
register_coco_instances("ball_train", {},"./Telstar_Mechta/test/_annotations.coco.json", "./Telstar_Mechta/test")
MetadataCatalog.get("ball_train").thing_classes = ['ball', 'robot']


# read image for inference input
# use PIL, to be consistent with evaluation
img = cv2.imread(img_path)
#img = cv2.resize(img, (1024, 1024))
img = torch.from_numpy(np.ascontiguousarray(img))
img = img.permute(2, 0, 1)  # HWC -> CHW
if torch.cuda.is_available():
    img = img.cuda()
inputs = [{"image": img}]

# run the model
model.eval()
with torch.no_grad():
    start = time.time()
    predictions_ls = model(inputs)
    print(f'Time: {time.time() - start}')
predictions = predictions_ls[0]

print(predictions)
im = cv2.imread(img_path)
#im = cv2.resize(im, (1024, 1024))
color = (255, 0, 0)

for i in range(len(predictions["instances"])):
	#print(predictions["instances"][i])
	#print(predictions["instances"][0].get("pred_boxes"))
	box = predictions["instances"][i].get("pred_boxes")
	box_tensor = box.tensor[0]
	x_inicial = box_tensor.data[0].item()
	y_inicial = box_tensor.data[1].item()
	x_final = box_tensor.data[2].item()
	y_final = box_tensor.data[3].item()
	start = (int(x_inicial), int(y_inicial))
	final = (int(x_final), int(y_final))
	score = predictions["instances"][i].get("scores").item()
	if (score>threshold):
		cv2.rectangle(im, start, final, color, 5)
cv2.imshow("RoboFEI",im)
cv2.waitKey(0)





