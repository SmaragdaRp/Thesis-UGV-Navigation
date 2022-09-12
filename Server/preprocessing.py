import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def image_preprocessing(filepath):
    image = cv2.imread(filepath) #reads image
    image = cv2.resize(image, (672,672), interpolation = cv2.INTER_AREA) #resize photo in 672x672

    # blur = cv2.medianBlur(image, 5)
    blur = cv2.GaussianBlur(image,(5,5),0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    # find white areas
    (thresh, binary_white) = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)

    # fill (closing) of dark regions in white areas
    kernel = np.ones(360)
    binary_white = cv2.morphologyEx(binary_white, cv2.MORPH_CLOSE, kernel)

    # mask only white rois from initial image
    masked_white_rois = cv2.bitwise_or(gray, 255-binary_white)

    # find black areas inside white areas of image
    masked_white_rois = cv2.medianBlur(masked_white_rois, 5)
    (thresh, binary_black) = cv2.threshold(255-masked_white_rois, 190, 255, cv2.THRESH_BINARY)

    contours = cv2.findContours(binary_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    result = image.copy()
    cntr_area = [cv2.contourArea(cntr) for cntr in contours]
    if not cntr_area:
        print(filepath)
    cntr_indx = np.argmax(cntr_area)
    # for cntr in contours:
    cntr = contours[cntr_indx]
    x,y,w,h = cv2.boundingRect(cntr)
    if x<60:
        x = x - (x//2)
    else:
        x -= 50
    if y<60:
        y = y - (y//2)
    else:
        y -= 50
    if w<50 or h<50:
        w=200
        h=200
    else:
        w=250
        h=250
    cv2.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)

    return image, (x, y, w, h)