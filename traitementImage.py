import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

cv2.WITH_QT = True

id = 0 
captureVideo = cv2.VideoCapture(id, cv2.CAP_DSHOW)
    
def getMousePosition (event, x, y, flags, param):
    global mousePos
    global iPressed
    global initDone
    if iPressed == True:
        if event == cv2.EVENT_LBUTTONDOWN:
            mousePos = [y, x]
            print(param[y, x])
            initDone = True
            print("Initialisation finished : color value = " + str(mousePos))
    

print("press i to start initialisation, the ball should be visible : ")

iPressed = False
initDone = False
postInit = False
mousePos = []

zoneL = []
zoneH = []

fig = plt.figure(1)
ax = fig.add_subplot()

#Initialisation
while 1:
    
    pressedKey = cv2.waitKey(1) & 0xFF
    
    ret, frame = captureVideo.read()
    
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
    cv2.imshow('camera', frame)
    
    H = hsv_img[:, :, 0]
    #cv2.imshow('cameraH', H)
    S = hsv_img[:, :, 1]
    #cv2.imshow('cameraS', S)
    V = hsv_img[:, :, 2]
    #cv2.imshow('cameraV', V)
        
    if initDone == False :
        if (pressedKey == ord('i')):
            print("Cliquer sur la balle")
            iPressed = True
        
            cv2.setMouseCallback('camera', getMousePosition, frame)
    
    if initDone == True :
        print("PostInit")
        zoneL = hsv_img[mousePos[0], mousePos[1], :] - [10, 10, 10]
        zoneH = hsv_img[mousePos[0], mousePos[1], :] + [10, 10, 10]
        postInit = True
        initDone = False
        
    if postInit == True:
        ret, Hbw = cv2.threshold(H, zoneH[0], 255, cv2.THRESH_BINARY) 
        ret, Hbw = cv2.threshold(H, zoneL[0], 255, cv2.THRESH_BINARY_INV) 
        
        ret, Sbw = cv2.threshold(S, zoneH[1], 255, cv2.THRESH_BINARY) 
        ret, Sbw = cv2.threshold(S, zoneL[1], 255, cv2.THRESH_BINARY_INV) 
        
        ret, Vbw = cv2.threshold(V, zoneH[2], 255, cv2.THRESH_BINARY) 
        ret, Vbw = cv2.threshold(V, zoneL[2], 255, cv2.THRESH_BINARY_INV) 
        
        cv2.imshow("Hbw", Hbw)
        cv2.imshow("Sbw", Sbw)
        cv2.imshow("Vbw", Vbw)
        
        kernel = np.ones((10, 10), np.uint8)
        for a in range(50):
            HbwE = cv2.dilate(Hbw, kernel)
        #cv2.imshow("HbwE", HbwE)
        for a in range(50):
            HbwED = cv2.erode(HbwE, kernel)
        
        cv2.imshow("HbwED", HbwED)
        HbwED = (255-HbwED)
        
        for a in range(2): #Nombre de balles à détecter
            dist = cv2.distanceTransform(HbwED, cv2.DIST_L2, 5);
            dist = cv2.normalize(dist, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cv2.imshow("dist", dist)
            a, b, c, d = cv2.minMaxLoc(dist)
            ax.scatter(d[0], d[1])
            fig.canvas.draw() 
            HbwED[d[1]-30: d[1]+30, d[0]-30: d[0]+30] = 0
        
        cv2.imshow("HbwED0", HbwED)
        ax.clear()
        ax.set_ylim([480,0])
        ax.set_xlim([640,0])
        
        

    
    
    #Fermeture des fenetres
    if(pressedKey == ord('q')):
        break
    
   
captureVideo.release()
cv2.destroyAllWindows()


