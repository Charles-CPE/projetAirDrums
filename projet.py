# Projet de fin d'annee : AirDrums

#Borde Charles
#Frossard Gabriel

import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import sys
import socket

from enum import Enum
from Camera import Camera
from Cameras import Cameras

#--------------------PARAMETRES------------------------

    #socket pour envoit d'info à Unity
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UDP_IP = "127.0.0.1"
UDP_PORT = 5065

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
coord_mm[:,1] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 1]
coord_mm[:,2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 1]
coord_mm *= 27
coord_mm[:,1] /= math.sqrt(2)
coord_mm[:,2] /= math.sqrt(2)
mireParams = [columns, rows, coord_mm]

    #Points à afficher à la fin (Boite donc la more est la base) #pour anle de 45°
coord_proj = np.array([[0, 0, 0], [297, 0, 0], [297, 162/math.sqrt(2), 162/math.sqrt(2)], [0, 162/math.sqrt(2), 162/math.sqrt(2)]], dtype = np.float32)



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

#figPositions = plt.figure(0)
#figPositions.suptitle("Positions des Cameras")
#axPositions = figPositions.add_subplot(projection='3d')



#-------------------------------CALIBRATION-------------------------------
calibrationDone = False
pressedKey = cv2.waitKey(1) & 0xFF
print("Appuyer sur c une fois la calibration satisfaisante \nOu p pour passer la calibration")

while (1):
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
        #axPositions.scatter(coord_proj[:, 0], coord_proj[:, 1], coord_proj[:, 2])
        
        #for idx in range(len(cameras.list)):
        #    axPositions.scatter(positions[idx][0], positions[idx][2], positions[idx][1])
        #axPositions.set_ylim([-1000,1000])
        #axPositions.set_xlim([-1000,1000])
        #axPositions.set_zlim([-1000,1000])

        #figPositions.canvas.draw()
        #axPositions.clear()
        
        #Projection de la boite sur la mire
        projections = cameras.projectionPoints(coord_proj)
        
        for idx in range(len(cameras.list)):
            for a in range(int(len(projections[0]))):
                frames[idx] = cv2.circle(frames[idx], (projections[idx][a,0,0], projections[idx][a,0,1]), 5, (255, 0, 0), -1)
                
        if(pressedKey == ord('c')):
            calibrationDone = True
            break
    else:
        calibrationDone = False

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
        #plt.close(figPositions)
        sys.exit()
        break
print(calibrationDone)
#plt.close(figPositions)

#-------------------------TRAITEMENT D'IMAGE-----------------------------
colorPick = 0
clickHSVList = []

def getMouseHSV (event, x, y, flags, param):
    global clickHSVList
    global colorPick
    
    if event == cv2.EVENT_LBUTTONDOWN:
        clickHSVList.append(param[y, x])
        colorPick += 1
                    
#Recuperation de la couleur de la balle en HSV
print("Cliquer sur les balles \n")
while colorPick != 2:
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
print(clickHSVList)

#bornes pour le threshold
marge = 10  
zoneLow1 = clickHSVList[0] - [marge, marge, marge]
zoneHigh1 = clickHSVList[0] + [marge, marge, marge]

zoneLow2 = clickHSVList[1] - [marge, marge, marge]
zoneHigh2 = clickHSVList[1] + [marge, marge, marge]

zonesLow = [zoneLow1, zoneLow2]
zonesHigh = [zoneHigh1, zoneHigh2]

#Figure d'affichage de l'axe
figAxe = plt.figure(0)
axAxe = figAxe.add_subplot(projection='3d')

#Fonction pour triangulation des balles
def DLT(P1, P2, point1, point2):
 
    A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:]
        ]
    A = np.array(A).reshape((4,4))
    #print('A: ')
    #print(A)
 
    B = A.transpose() @ A
    from scipy import linalg
    U, s, Vh = linalg.svd(B, full_matrices = False)
    
    return Vh[3,0:3]/Vh[3,3]
    
lastHueFrameBW = frames[0][:, :, 0]
#Détection des balles
while 1:
    pressedKey = cv2.waitKey(1) & 0xFF
    positionsBalles = []
    
    rets, frames = cameras.capture()
    
    imageCameras = cv2.hconcat(frames)
    cv2.imshow('Calibration des cameras', imageCameras)
    
    for idx in range(len(cameras.list)):
        resultsCamera = []
        positionBalle = []
        hsvFrame = cv2.cvtColor(frames[idx], cv2.COLOR_BGR2HSV)
    
        #Hue
        hueFrame = hsvFrame[:, :, 0]
        hueFrameSmall = cv2.resize(hueFrame,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    
        for balleIdx in range(2):
            #Thershold
            ret, hueFrameBWH = cv2.threshold(hueFrame, zonesLow[balleIdx][0], 255, cv2.THRESH_BINARY) 
            ret, hueFrameBWL = cv2.threshold(hueFrame, zonesHigh[balleIdx][0], 255, cv2.THRESH_BINARY_INV)
            hueFrameBW = hueFrameBWH & hueFrameBWL
            
            # if idx == 10:
            #     hueFrameBWStock = hueFrameBW
            #     hueFrameBW = hueFrameBW | lastHueFrameBW
            #     lastHueFrameBW = hueFrameBWStock
                
            hueFrameBWSmall = cv2.resize(hueFrameBW,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    
            #Ouverture morphologique
            kernel = np.ones((3, 3), np.uint8)
            
            hueFrameBWOuv = hueFrameBW
            for a in range(5):
                hueFrameBWOuv = cv2.erode(hueFrameBWOuv, kernel)
            for a in range(15):
                hueFrameBWOuv = cv2.dilate(hueFrameBWOuv, kernel)
            hueFrameBWOuvSmall = cv2.resize(hueFrameBWOuv,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
            
            #Carte des distances
            hueFrameBWOuvDist = cv2.distanceTransform(hueFrameBWOuv, cv2.DIST_L2, 5);
            hueFrameBWOuvDist = cv2.normalize(hueFrameBWOuvDist, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            #recherche du max
            a, b, c, d = cv2.minMaxLoc(hueFrameBWOuvDist) #d coordonndées du max
            positionBalle.append([d[0], d[1]])
            
            #hueFrameBWOuv[d[1]-30: d[1]+30, d[0]-30: d[0]+30] = 0
            
            hueFrameBWOuvDistSmall = cv2.resize(hueFrameBWOuvDist,(320, 240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
            
            resultBalle = cv2.hconcat([hueFrameSmall, hueFrameBWSmall, hueFrameBWOuvSmall, hueFrameBWOuvDistSmall])
            resultsCamera.append(resultBalle)
        resultsCamera = cv2.vconcat([resultsCamera[0], resultsCamera[1]])
        cv2.imshow("resultCamera : " + str(idx), resultsCamera)
        
        #[[[cam0b0x, cam0b0y], [cam0b1x, cam0b1y]], [[cam1b0x, cam1b0y], [cam1b1x, cam1b1y]]]
        positionsBalles.append(positionBalle)
        
    #print(positionsBalles)
    


    #Calcul de la position dans l'espace
    if calibrationDone == True:
        positionsBalles3D = []
        if (([0, 0] not in positionsBalles[0]) & ([0, 0] not in positionsBalles[1])):
            for balleIdx in range(2):
                positionBalle3D = DLT(camera1.projMat, camera2.projMat, positionsBalles[0][balleIdx], positionsBalles[1][balleIdx])
                positionsBalles3D.append(positionBalle3D)
                
            #Envoie des données à Unity
            txt = ""
            balles = [int(positionsBalles3D[0][0]), int(positionsBalles3D[0][1]), int(positionsBalles3D[0][2]), int(positionsBalles3D[1][0]), int(positionsBalles3D[1][1]), int(positionsBalles3D[1][2])]
            for coord in balles:
                txt += str(coord) + ","
            sock.sendto( (txt).encode(), (UDP_IP, UDP_PORT) )
            #print("_"*10, "Envoit des coordonnées 3D camera : ", str(idx), "_"*10)
            #print(positionsBalles3D[0][0], positionsBalles3D[0][1], positionsBalles3D[0][2], positionsBalles3D[1][0], positionsBalles3D[1][1], positionsBalles3D[1][2])
            print(positionsBalles)
            print(txt)
            print("\n")
            
    if(pressedKey == ord('q')):
        cameras.release()
        cv2.destroyAllWindows()
        sys.exit()
        break
    
    
cameras.release()
cv2.destroyAllWindows()