import cv2
from dataclasses import dataclass

def draw_lines(img, config):

    # Draw horizontal divisions
    cv2.line(img, (config.x_left, 0), (config.x_left, img.shape[0]), (0, 0, 255), 1)
    cv2.line(img, (config.x_center, 0), (config.x_center, img.shape[0]), (0, 0, 255), 1)
    cv2.line(img, (config.x_right, 0), (config.x_right, img.shape[0]), (0, 0, 255), 1)

    # Draw vertical divisions
    cv2.line(img, (0, config.y_chute), (img.shape[1], config.y_chute), (0, 0, 255), 1)
    cv2.line(img, (0, config.y_longe), (img.shape[1], config.y_longe), (0, 0, 255), 1)

    return img

def recise_image(img, scale_percent=100):

    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    
    return resized

@dataclass
class position:
    x: int = 0
    y: int = 0