import socket
import numpy as np
import cv2

class Client():
    def __init__(self, ip = '192.168.7.10', port = 5050, buffer_size = 40960):
        self.ip = ip
        self.port = port
        self.addr = (self.ip, self.port)
        self.buffer_size = buffer_size
        self.__open_socket()

    def __open_socket(self):
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def send_image(self, img):
        # Codifica o frame em JPEG
        _, img_encoded = cv2.imencode('.jpg', img)
        data = img_encoded.tobytes()

        # Envia o tamanho da imagem primeiro
        # Envia como string para difernciar dos dados da camera
        flag = f'{len(data)}'
        self.udp.sendto(flag.encode(), self.addr)

        # Envia a imagem em partes
        for i in range(0, len(data), self.buffer_size):
            self.udp.sendto(data[i:i + self.buffer_size], self.addr)
    
    def close_socket(self):
        self.udp.close()
