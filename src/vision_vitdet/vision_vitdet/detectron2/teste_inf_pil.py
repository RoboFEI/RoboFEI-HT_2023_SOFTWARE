#!/usr/bin/env python3
import numpy as np
import torch
import cv2
from telnetlib import NOP
from PIL import Image

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import LazyConfig, instantiate
from detectron2.data.detection_utils import read_image
from detectron2.data import MetadataCatalog
from detectron2.utils.visualizer import ColorMode
from detectron2.data.datasets import register_coco_instances
import glob
import time

img_path = "./bola.jpg"
threshold = 0.75

cfg = LazyConfig.load("./projects/ViTDet/configs/COCO/mask_rcnn_vitdet_b_100ep.py")

# edit the config to utilize common Batch Norm
cfg.model.backbone.norm = "BN"
cfg.model.roi_heads.num_classes = 1

model = instantiate(cfg.model)

DetectionCheckpointer(model).load("./train_ball_5000/model_final.pth")  # load a file, usually from cfg.MODEL.WEIGHTS
register_coco_instances("ball_train", {},"./train/_annotations.coco.json", "./train")
MetadataCatalog.get("ball_train").thing_classes = ['ball']
# run the model
model.eval()

filename = "./bola.jpg"
image = Image.open(filename)
image = np.array(image, dtype=np.uint8)
image = np.moveaxis(image, -1, 0) # the model expects the image to be in channel first format

with torch.inference_mode():
	start = time.time()
	output = model([{'image': torch.from_numpy(image)}])
	print(f'Time: {time.time() - start}')


predictions = output[0]

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
cv2.imshow("bola",im)
cv2.waitKey(0)





