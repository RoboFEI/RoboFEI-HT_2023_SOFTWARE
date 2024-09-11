import os
import datetime
import time
import cv2

class ImageGetter():
    def __init__(self, folderPath, fps):
        today = datetime.datetime.now()
        self.vision_log_path = f'{folderPath}/{today.day}_{today.month}_{today.year}-{today.hour}_{today.minute}'
        os.makedirs(self.vision_log_path, exist_ok=True)
        self.old_time = 
    
    def save_image(self, img):
        if time.time() - self.old_time > 1.0/self.fps_save:
            self.old_time = time.time()
            file_name = f"/ball_photo{self.foto_count:04d}.jpg"
            cv2.imwrite(self.vision_log_path+file_name, img)
            self.foto_count += 1