# Projet de fin d'annee : AirDrums

#Borde Charles
#Frossard Gabriel

import numpy as np
import cv2
import matplotlib.pyplot as plt
import math as m

from enum import Enum
from Camera import Camera



#parametres intraseques aux camameras
class C270(Enum):   #acces : C270.__.value
    w = 1280
    h = 960
    f = 4*(10**-3)
    s1 = 3.58*(10**-3)/w
    s2 = 2.02*(10**-3)/h



#parametres de la mire
columns = 12
rows = 7
coord_mm = np.zeros((columns*rows,3), np.float32)
coord_mm[:,0] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 0]
coord_mm[:,2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)[:, 1]
coord_mm *= 20

mireParams = [columns, rows, coord_mm]



#Initialisation des cameras
camera1 = Camera(1, C270, mireParams)
camera1.initialisation()
camera2 = Camera(2, C270, mireParams)
camera2.initialisation()



#Points à afficher à la fin
coord_proj = np.array([[0, 0, 0], [240, 0, 0], [240, 0, 140], [0, 0, 140], [0, 140, 0], [240, 140, 0], [0, 140, 140], [240, 140, 140]], dtype = np.float32)



#fenetres d'affichage de données
plt.ion()

fig1 = plt.figure(1)
ax1 = fig1.add_subplot(projection='3d')

fig2 = plt.figure(2)
ax2 = fig2.add_subplot()

while (True):
    ret1, frame1 = camera1.capture()
    ret2, frame2 = camera2.capture()
    
    
    #calibration
    if (ret1 & ret2):
        ret1, frame1 = camera1.calibrate(frame1)
        ret2, frame2 = camera2.calibrate(frame2)
        
    #recuperation des positions des caméras puis affichage
    if (ret1 & ret2):
        #print(camera1.tvecs)
        #print(camera1.rvecs)
        #print(camera1.dist)
        
        R1, _ = cv2.Rodrigues(camera1.rvecs)
        R1 = np.transpose(R1)
        cameraPos1 = -np.matmul(R1, camera1.tvecs)
        
        R2, _ = cv2.Rodrigues(camera2.rvecs)
        R2 = np.transpose(R2)
        cameraPos2 = -np.matmul(R2, camera2.tvecs)
        
        #Affichage des positions 3D
        ax1.scatter(coord_proj[:, 0], coord_proj[:, 1], coord_proj[:, 2])
        ax1.scatter(cameraPos1[0], cameraPos1[2], cameraPos1[1])
        ax1.scatter(cameraPos2[0], cameraPos2[2], cameraPos2[1])
        ax1.set_ylim([-1000,1000])
        ax1.set_xlim([-1000,1000])
        ax1.set_zlim([-1000,1000])

        fig1.canvas.draw()
        ax1.clear()
        
        #Distance entre les deux caméras
        dist = [abs(cameraPos1[0] - cameraPos2[0]), abs(cameraPos1[1] - cameraPos2[1]), abs(cameraPos1[2] - cameraPos2[2])]
        dist = np.sqrt(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2])
        print(dist)
        #print(cameraPos1)
        
        proj1 = camera1.projectionPoints(coord_proj)
        proj2 = camera2.projectionPoints(coord_proj)
        
        for a in range(int(len(proj1))):
            frame1 = cv2.circle(frame1, (proj1[a,0,0], proj1[a,0,1]), 5, (255, 0, 0), -1)
            frame2 = cv2.circle(frame2, (proj2[a,0,0], proj2[a,0,1]), 5, (255, 0, 0), -1)
    camera1.show(frame1, 'findChess1')    
    camera2.show(frame2, 'findChess2')    
    
    #Fermeture des fenetres
    if(cv2.waitKey(1) & 0xFF == ord('q')):
       break
    
camera1.release()
camera2.release()


cv2.destroyAllWindows()