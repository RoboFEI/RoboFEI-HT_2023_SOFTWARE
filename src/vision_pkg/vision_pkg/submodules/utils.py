import cv2
from dataclasses import dataclass
import numpy as np

from .image import drawBallBox

def draw_lines(img, config):

    # Tamanho da imagem
    height, width = img.shape[:2]

    x_left = width // 3
    x_right = (width * 2) // 3

    cv2.line(img, (x_left, 0), (x_left, height), (0, 0, 255), 1)
    cv2.line(img, (x_right, 0), (x_right, height), (0, 0, 255), 1)

    y_top = height // 3
    y_bottom = (height * 2) // 3

    cv2.line(img, (0, y_top), (width, y_top), (0, 0, 255), 1)
    cv2.line(img, (0, y_bottom), (width, y_bottom), (0, 0, 255), 1)

    return img



@dataclass
class position:
    x: int = 0
    y: int = 0