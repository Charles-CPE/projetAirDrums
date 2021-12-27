# Projet de fin d'annee : AirDrums

#Borde Charles
#Frossard Gabriel

import numpy as np
import cv2
import matplotlib.pyplot as plt

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
coord_mm[:,:2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)
coord_mm *= 20

mireParams = [columns, rows, coord_mm]

#Initialisation des cameras

camera1 = Camera(1, C270, mireParams)
camera1.initialisation()


#Points à afficher à la fin
#coord_proj = np.array([[0, 0, 0], [0, 0, 20], [0, 0, 40], [0, 0, 60], [0, 0, 100], [0, 0, 200], [0, 0, 400], [0, 0, 800]])
coord_proj = np.array([[0, 0, 0], [240, 0, 0], [240, 140, 0], [0, 140, 0], [0, 0, -140], [240, 0, -140], [0, 140, -140], [240, 140, -140]], dtype = np.float32)


while (True):
    ret, frame = camera1.capture()
    #camera1.show(frame, 'init')   
    if (ret):
        ret, frame = camera1.calibrate(frame)
      
    if (ret):
        print(camera1.tvecs)
        #print(camera1.rvecs)
        #print(camera1.dist)
        
        proj = camera1.projectionPoints(coord_proj)
        for a in range(int(len(proj))):
            frame = cv2.circle(frame, (proj[a,0,0], proj[a,0,1]), 5, (255, 0, 0), -1)

        
    camera1.show(frame, 'findChess')    

    
    #Fermeture des fenetres
    if(cv2.waitKey(1) & 0xFF == ord('q')):
       break
    
camera1.release()

def AffichageMultiple (nombre, liste):
    
    return 1


cv2.destroyAllWindows()