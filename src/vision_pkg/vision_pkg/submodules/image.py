import cv2
import numpy as np

def drawBallBox(img, results, ball_index):
    ball_pos =  np.array([])
    img_cp = img.copy()

    if ball_index == -1:
        return img_cp, ball_pos
    
    ball_box_xywh = results.boxes[ball_index].xywhn.numpy() #get the most conf ball detect box in xyxy format
    array_box_xywh = np.reshape(ball_box_xywh, -1)  #convert matriz to array

    ball_pos = np.array([0, 0], dtype=float)
    ball_pos[0] = float(array_box_xywh[0] * img.shape[1])
    ball_pos[1] = float(array_box_xywh[1] * img.shape[0])
    
    raio_ball   = int((array_box_xywh[2] * img.shape[1] +array_box_xywh[3] * img.shape[0]) / 4)

    cv2.circle(img_cp, (int(ball_pos[0]), int(ball_pos[1])), abs(raio_ball), (255, 0, 0), 2)
    cv2.circle(img_cp, (int(ball_pos[0]), int(ball_pos[1])), 5, (255, 0, 0), -1)

    return img_cp, ball_pos

def resize_image(img, scale_percent=100):

    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    
    return resized

def findBall(img, results, classesValues):
    img_cp = img.copy()
    
    # Return the most confidence ball index, if ball not found index equal -1
    try:
        ball_index = np.where(results.boxes.cls == classesValues['ball'])[0][0]
    except: 
        ball_index = -1
    
    img_cp, ballPositionPx = drawBallBox(img_cp, results, ball_index)
        
    return img_cp, ballPositionPx
 