import socket
import numpy as np
import cv2

# Configurações do servidor UDP
SERVER_IP = ""
SERVER_PORT = 5050
BUFFER_SIZE = 40960

# Configura o socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, SERVER_PORT))

print("Servidor iniciado. Aguardando imagens.")

while True:
    try:
        size_msg, add = sock.recvfrom(BUFFER_SIZE)
        size_msg.decode()
    except:
        continue

    size = int(size_msg)

    # Recebe a imagem em partes
    img_data = b''
    while len(img_data) < size:
        packet, addr = sock.recvfrom(BUFFER_SIZE)
        
        try:
            packet.decode()
            break
        except:
            pass
        
        img_data += packet
    
    if(size != len(img_data)):
        continue

    # Decodifica a imagem
    np_arr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is not None:
        cv2.imshow('Received Image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Erro ao decodificar a imagem.")

# Fecha as janelas e o socket
cv2.destroyAllWindows()
sock.close()
