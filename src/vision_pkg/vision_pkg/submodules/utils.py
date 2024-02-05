import cv2
import numpy as np

def draw_boxes(img, results, value_classes):

    img_copy = img.copy()

    ball_detection = (results.boxes.cls == value_classes['ball']).nonzero(as_tuple=True)[0].numpy()

    if ball_detection.size > 0:
        ball_box_xyxy = results.boxes[ball_detection[0]].xyxy.numpy() #get the most conf ball detect box in xyxy format
        array_box_xyxy = np.reshape(ball_box_xyxy, -1)  #convert matriz to array

        ball_center_pos = (int((array_box_xyxy[0] + array_box_xyxy[2]) / 2), #X position of the ball in pixels
                          int((array_box_xyxy[1] + array_box_xyxy[3]) / 2))  #Y position of the ball in pixels
        
        raio_ball       = int((array_box_xyxy[2] - array_box_xyxy[0]) / 2)
        cv2.circle(img_copy, ball_center_pos, abs(raio_ball), (255, 0, 0), 2)
        cv2.circle(img_copy, ball_center_pos, 5, (255, 0, 0), -1)
    
    return img_copy

def draw_lines(img, config):

    # Draw horizontal divisions
    cv2.line(img, (config.x_left, 0), (config.x_left, img.shape[0]), (0, 0, 255), 1)
    cv2.line(img, (config.x_center, 0), (config.x_center, img.shape[0]), (0, 0, 255), 1)
    cv2.line(img, (config.x_right, 0), (config.x_right, img.shape[0]), (0, 0, 255), 1)

    # Draw vertical divisions
    cv2.line(img, (0, config.y_chute), (img.shape[1], config.y_chute), (0, 0, 255), 1)
    cv2.line(img, (0, config.y_longe), (img.shape[1], config.y_longe), (0, 0, 255), 1)

    return img

def locate_ball(img, results):
    pass