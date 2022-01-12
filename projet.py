# Projet de fin d'annee : AirDrums

#Borde Charles
#Frossard Gabriel

import numpy as np
import cv2
import matplotlib.pyplot as plt
import math as m
import sys

from enum import Enum
from Camera import Camera
from Cameras import Cameras

#--------------------PARAMETRES------------------------

    #Parametres intraseques aux camameras
class C270(Enum):   #acces : C270.__.value
    w = 640
    h = 480
    f = 4*(10**-3)
    s1 = 3.58*(10**-3)/w
    s2 = 2.02*(10**-3)/h

    #Parametres de la mire
columns = 12
rows = 7
coord_mm = np.zeros((columns*rows,3), np.float32)
coord_mm[:,0] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 0]
coord_mm[:,2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 1]
coord_mm *= 20
mireParams = [columns, rows, coord_mm]

    #Points à afficher à la fin (Boite donc la more est la base)
coord_proj = np.array([[0, 0, 0], [240, 0, 0], [240, 0, 140], [0, 0, 140], [0, 140, 0], [240, 140, 0], [0, 140, 140], [240, 140, 140]], dtype = np.float32)



#----------------------INITIALISATION------------------------
    #Creation des cameras
camera1 = Camera(0, C270, mireParams)
camera2 = Camera(1, C270, mireParams)

    #Creation du groupe Cameras
cameras = Cameras(mireParams, [camera1, camera2])

    #Initialisation des caméras (VideoCapture)
cameras.initialisation()



#-----------------------PREPARATION AFFICHAGE-----------------------(A REFAIRE)
#fenetres d'affichage de données
plt.ion() #Turn the interactive mode On

figPositions = plt.figure(0)
figPositions.suptitle("Positions des Cameras")
axPositions = figPositions.add_subplot(projection='3d')



#-------------------------------CALIBRATION-------------------------------
calibrationDone = True
pressedKey = cv2.waitKey(1) & 0xFF
print("Appuyer sur c une fois la calibration satiafaisante \nOu p pour passer la calibration")

while (pressedKey != ord('c')):
    pressedKey = cv2.waitKey(1) & 0xFF
    
    rets, frames = cameras.capture()

    if (sum(rets) == len(rets)):
        rets, frames = cameras.calibrate(frames)
        
        for idx in range(len(cameras.list)):
            cv2.putText(frames[idx], "camera : " + str(idx), (250, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_4)

    #recuperation des positions des caméras puis affichage
    if (sum(rets) == len(rets)):
        
        positions = cameras.camerasPos()

        #Affichage des positions 3D
        axPositions.scatter(coord_proj[:, 0], coord_proj[:, 1], coord_proj[:, 2])
        axPositions.scatter(positions[0][0], positions[0][2], positions[0][1])
        axPositions.scatter(positions[1][0], positions[1][2], positions[1][1])
        axPositions.set_ylim([-1000,1000])
        axPositions.set_xlim([-1000,1000])
        axPositions.set_zlim([-1000,1000])

        figPositions.canvas.draw()
        axPositions.clear()
        
        #Projection de la boite sur la mire
        projections = cameras.projectionPoints(coord_proj)
        
        for idx in range(len(cameras.list)):
            for a in range(int(len(projections[0]))):
                frames[idx] = cv2.circle(frames[idx], (projections[idx][a,0,0], projections[idx][a,0,1]), 5, (255, 0, 0), -1)

    imageCameras = cv2.hconcat(frames)
    cv2.imshow('Calibration des cameras', imageCameras)
    
    #Passer la calibration
    if(pressedKey == ord('p')):
        calibrationDone  = False
        break
    
    #Fermeture des fenetres
    if(pressedKey == ord('q')):
        cameras.release()
        cv2.destroyAllWindows()
        plt.close(figPositions)
        sys.exit()
        break
print(calibrationDone)
plt.close(figPositions)

#-------------------------TRAITEMENT D'IMAGE-----------------------------
colorPick = False
clickHSV = []

def getMouseHSV (event, x, y, flags, param):
    global clickHSV
    global colorPick
    
    if event == cv2.EVENT_LBUTTONDOWN:
        clickHSV = param[y, x]
        colorPick = True
                    
#Recuperation de la couleur de la balle en HSV
print("Cliquer sur la balle sur l'une des caméras \n")
camera2.initialisation()
while colorPick == False:
    pressedKey = cv2.waitKey(1) & 0xFF
    
    rets, frames = cameras.capture()
    imageCameras = cv2.hconcat(frames)
    cv2.imshow('Calibration des cameras', imageCameras)
    
    hsvImageCameras = cv2.cvtColor(imageCameras, cv2.COLOR_BGR2HSV)
    cv2.setMouseCallback('Calibration des cameras', getMouseHSV, hsvImageCameras)
    
    if(pressedKey == ord('q')):
        cameras.release()
        cv2.destroyAllWindows()
        sys.exit()
        break
    
print("HSV de la balle : ")
print(clickHSV)

#bornes pour le threshold
marge = 10
zoneLow = clickHSV - [marge, marge, marge]
zoneHigh = clickHSV + [marge, marge, marge]


#Détection des balles
while 1:
    pressedKey = cv2.waitKey(1) & 0xFF
    positionsBalles = []
    
    rets, frames = cameras.capture()
    imageCameras = cv2.hconcat(frames)
    cv2.imshow('Calibration des cameras', imageCameras)
    
    for idx in range(len(cameras.list)):
        positionBalle = []
        hsvFrame = cv2.cvtColor(frames[idx], cv2.COLOR_BGR2HSV)
    
        #Hue
        hueFrame = hsvFrame[:, :, 0]
        hueFrameSmall = cv2.resize(hueFrame,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    
        #Thershold
        ret, hueFrameBWH = cv2.threshold(hueFrame, zoneLow[0], 255, cv2.THRESH_BINARY) 
        ret, hueFrameBWL = cv2.threshold(hueFrame, zoneHigh[0], 255, cv2.THRESH_BINARY_INV)
        hueFrameBW = hueFrameBWH & hueFrameBWL
        hueFrameBWSmall = cv2.resize(hueFrameBW,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)

        #Ouverture morphologique
        kernel = np.ones((3, 3), np.uint8)
        
        hueFrameBWOuv = hueFrameBW
        for a in range(5):
            hueFrameBWOuv = cv2.erode(hueFrameBWOuv, kernel)
        for a in range(5):
            hueFrameBWOuv = cv2.dilate(hueFrameBWOuv, kernel)
        hueFrameBWOuvSmall = cv2.resize(hueFrameBWOuv,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
        
        for a in range(2): #Nombre de balles à détecter
            hueFrameBWOuvDist = cv2.distanceTransform(hueFrameBWOuv, cv2.DIST_L2, 5);
            hueFrameBWOuvDist = cv2.normalize(hueFrameBWOuvDist, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            a, b, c, d = cv2.minMaxLoc(hueFrameBWOuvDist) #d coordonndées du max
            positionBalle.append([d[0], d[1]])
            hueFrameBWOuv[d[1]-30: d[1]+30, d[0]-30: d[0]+30] = 0
        positionsBalles.append(positionBalle)
        hueFrameBWOuvDistSmall = cv2.resize(hueFrameBWOuvDist,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
            
        resultTest = cv2.hconcat([hueFrameSmall, hueFrameBWSmall, hueFrameBWOuvSmall, hueFrameBWOuvDistSmall])
        cv2.imshow("resultCamera : " + str(idx), resultTest)
        
    #print(positionsBalles)
    
    #Calcul de la position dans l'espace
    if calibrationDone == True:
        print("calcul des coordonnées 3D")
            
    if(pressedKey == ord('q')):
        cameras.release()
        cv2.destroyAllWindows()
        sys.exit()
        break

#print(mouseHSV)
#for frame in frames:
#    hsvFrame.append(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    
    
cameras.release()
cv2.destroyAllWindows()