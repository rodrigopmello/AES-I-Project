import os
import re
import cv2
import numpy as np
from os.path import isfile, join
import matplotlib.pyplot as plt


# get file names of the frames
col_frames = os.listdir('teste1/')

# sort file names
col_frames.sort(key=lambda f: int(re.sub('\D', '', f)))

# empty list to store the frames
col_images=[]

for i in col_frames:
    # read the frames
    img = cv2.imread('test1/'+i)
    # append the frames to the list
    
    col_images.append(img[604:1068, 836:1175])

# kernel for image dilation
kernel = np.ones((4,4),np.uint8)

# font style
font = cv2.FONT_HERSHEY_SIMPLEX

# directory to save the ouput frames
pathIn = "output/"

for i in range(len(col_images)-1):
    
    # frame differencing
    grayA = cv2.cvtColor(col_images[i], cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(col_images[i+1], cv2.COLOR_BGR2GRAY)
    plt.imshow(cv2.cvtColor(col_images[i], cv2.COLOR_BGR2RGB))
    plt.title("frame: "+str(i))
    plt.show()
    diff_image = cv2.absdiff(grayB, grayA)
    
    # image thresholding
    ret, thresh = cv2.threshold(diff_image, 30, 255, cv2.THRESH_BINARY)

    # plot image after thresholding
    plt.imshow(thresh, cmap = 'gray')
    plt.show()
    
    # image dilation
    dilated = cv2.dilate(thresh,kernel,iterations = 1)

    # plot dilated image
    plt.imshow(dilated, cmap = 'gray')
    plt.show()
    
    # find contours
    contours, hierarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    # shortlist contours appearing in the detection zone
    valid_cntrs = []
    for cntr in contours:
        x,y,w,h = cv2.boundingRect(cntr)
        if (x <= 200) & (y >= 300) & (cv2.contourArea(cntr) >= 25):
            if (y >= 300) & (cv2.contourArea(cntr) < 40):
                break
            valid_cntrs.append(cntr)
            
    # add contours to original frames
    dmy = col_images[i].copy()
    cv2.drawContours(dmy, valid_cntrs, -1, (127,200,0), 2)
    print("vehicles detected: "+  str(len(valid_cntrs)))
    cv2.putText(dmy, "vehicles detected: " + str(len(valid_cntrs)), (55, 15), font, 0.6, (0, 180, 0), 2)
    cv2.line(dmy, (0, 300),(256,300),(100, 255, 255))
    cv2.imwrite(pathIn+str(i)+'.png',dmy)  