# 1. Import libraries

import numpy as np
import cv2
import tensorflow
import preprocessing
import socket
import time


# 2. Load model

print("Loading VGG model...")

# VGG16 CatCrossentropy-Adam
model = tensorflow.keras.models.load_model('signs_vgg16.h5')

# VGG16 CatCrossentropy-Adagrad
# model = tensorflow.keras.models.load_model('signs_vgg16_adagrad.h5')


# # 3. Accept connection from Turtlebot

HOST = '0.0.0.0'
# HOST = '127.0.0.1'
PORT = 9898
BUFFER_SIZE = 2048

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))

server.listen()

print("I can now accept requests from clients.")

client_socket, client_addr = server.accept()

print(f"Client with address {client_addr} is connected.")


counter = 1
while True:

    # 4. Receive photo

    received_image = f"received_images/received{counter}.jpeg"

    with open(received_image, 'wb') as file:
        recv_data = client_socket.recv(BUFFER_SIZE)
        while recv_data:
            
            file.write(recv_data)
            #print(recv_data)
            if b"edima" in recv_data:
                break
            recv_data = client_socket.recv(BUFFER_SIZE)
    file.close()

    # 5. Preprocess photo
    
    result, rect = preprocessing.image_preprocessing(received_image)
    X=rect[0]
    Y=rect[1]
    W=rect[2]
    H=rect[3]
    cropped_image = result[Y:Y+H, X:X+W]
    cropped_image = cv2.resize(cropped_image, (224,224), interpolation = cv2.INTER_AREA) #resize photo in 224x224
    cv2.imwrite(f"cropped_images/result{counter}.jpeg", cropped_image)

    # 6. Predict class of photo

    y_pred=model.predict(np.expand_dims(cropped_image, axis=0))
    y_pred=np.argmax(y_pred,axis=1)
    # print(y_pred)


    # 7. Send result to Turtlebot

    if y_pred == [0]:
        client_socket.send(b"0")
        print("Stop.")
        break
    elif y_pred == [1]:
        client_socket.send(b"1")
        print("Go backwards.")
        time.sleep(3)
    elif y_pred == [2]:
        client_socket.send(b"2")
        print("Go forward.")
        time.sleep(2)
    elif y_pred == [3]:
        client_socket.send(b"3")
        print("Turn left.")
        time.sleep(5)
    else:
        client_socket.send(b"4")
        print("Turn right.")
        time.sleep(5)

    counter += 1


print(f"Images received: {counter}.")
print("Closing socket and exiting...")
file.close()
client_socket.close()
