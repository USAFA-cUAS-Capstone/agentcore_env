import cv2 as cv
import numpy as np
from timeit import default_timer as timer

#This section is to hard code an image to process before we have a live feed.

ImageFile = "C:\\Users\\cUAS_Laptop1\\Desktop\\ImageProc\\Krusty.jpg"

#Turn this to 1 for live processing
live = 0


def Controller():
    threshold = 100
    i = 0
    # frameNumber = 0
    # avgAdder = 0


    if(live == 1):
        cam = cv.VideoCapture(i)
        while(i < 5):    
            if(cam.isOpened() == True):
                break
            i = i+1
        if(cam.isOpened() == False):
            print("Could not open camera. Try again dummy.") 
            
    # start = timer()
    if(live == 1):
        ret, img = cam.read()
    else:
        img = cv.imread(ImageFile)

    invert = cv.bitwise_not(img)
    edgeHeight = img.shape[0]
    edgeWidth = img.shape[1]

    gray_image = cv.cvtColor(img, cv.COLOR_BGR2GRAY)    #Grayscale
    blurred = cv.GaussianBlur(gray_image, (7, 7), 0)    #Blur Images
    # blurred = cv.Canny(gray_image, 50,200)
    ret,thresh = cv.threshold(blurred, threshold,255,cv.ADAPTIVE_THRESH_MEAN_C)    #Convert to Binary
    contours,hierarchy = cv.findContours(thresh, 1, 2) #Find Contours
    cnts = contours[0]

    if(len(contours) > 0):
        j = 0
        sorted_contours=sorted(contours, key=cv.contourArea, reverse= True)
        
        while(j < len(contours)):
            largest_item= sorted_contours[j]
            (bigx, bigy, bigw, bigh) = cv.boundingRect(largest_item)
            j = j+1
            
            if not(bigx == 0 or bigy == 0 or bigw+bigx == edgeWidth or bigh+bigy == edgeHeight):
                break
                
            else:
                banana = 1

    cv.drawContours(img, largest_item, -1, (255,0,0),10)
    # cv.drawContours(img, cnts, -1, (0, 255, 0), 2)
    (x,y),radius = cv.minEnclosingCircle(largest_item)
    center = (int(x),int(y))
    radius = int(radius)
    cv.circle(img,center,radius,(0,0,255),2)

    M = cv.moments(thresh)

    # # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])


    #this might get checked earlier to get rid of edge contours
    height = int(img.shape[0])
    heightfor = int(height/2)
    width = int(img.shape[1]/2)

    #Determine Yaw
    Yaw = 0
    Yaw = cX - width
    Yaw = Yaw*.0359

    #Determine Z
    Alt = 0
    Alt = cY - (height/2 + height*.1)
    Alt = Alt * 1 #Change the constant to fine tune altitude control

    #print(cv.contourArea(contours))
    cv.line(img, (width,heightfor), center, (255,255,0), 4)
    cv.putText(img, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv.putText(img, str(Yaw), (height-10, width-50),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv.putText(img, str(Alt), (height-10, width-25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # cv.imshow('ret', thresh)

    # cv.imshow('this', thresh)
    # cv.imshow('me', img)

    # Debugging thing to use on ground
    # k = cv.waitKey(1)
    # if k%256 == 27:
    #     # ESC pressed
    #     print("Escape hit, closing...")
    #     break
    # elif (k%256 >= 49 and k%256<60):
    #     # Num pressed
    #     threshold = int(k-48)*20
    #     print("Threshold is now:")
    #     print(threshold)
    #     print("\n")
    # elif k%256 == 32:
    #     #Space Pressed
    #     print("Status:")
    #     print("Not implemented")
        
        
    # frameNumber = frameNumber + 1
    # avgAdder = avgAdder + (timer()-start)

    if(live == 1):
        cam.release()

    cv.destroyAllWindows()

    return [Yaw, Alt]

    # print("My program took:", avgAdder/frameNumber, "seconds to run a loop")