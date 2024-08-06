# import socket

# class ServerUDP():
#     def __init__(self):
#         self.ip = '0.0.0.0'
#         self.port = 5000
#         self.addr = (self.ip, self.port)
#         self._open_socket()

#     def _open_socket(self):
#         self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.udp.bind(self.addr)

#     def recive_once(self):
#         mensage, end_client = self.udp.recvfrom(1024)
#         print("Recebi = ", mensage, " , Do cliente", end_client)
    
#     def close_socket(self):
#         self.udp.close()
    
# server_udp = ServerUDP()
# server_udp.recive_once()
# server_udp.close_socket()







import socket
import numpy as np
import cv2

# Configurações do servidor UDP
SERVER_IP = '192.168.7.10'
SERVER_PORT = 12345
BUFFER_SIZE = 40960

# Configura o socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, SERVER_PORT))

print("Servidor iniciado. Aguardando imagens.")

while True:
    # Recebe o tamanho da imagem
    size_data, addr = sock.recvfrom(4)
    size = int.from_bytes(size_data, byteorder='big')

    # Recebe a imagem em partes
    img_data = b''
    while len(img_data) < size:
        packet, addr = sock.recvfrom(BUFFER_SIZE)
        img_data += packet

    # Decodifica a imagem
    np_arr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is not None:
        # Exibe a imagem recebida
        cv2.imshow('Received Image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Erro ao decodificar a imagem.")

# Fecha as janelas e o socket
cv2.destroyAllWindows()
sock.close()
