#Classe Camera

import cv2
import numpy as np

#Attributs:
    #w
    #h
    #id             Port de la camera
    #captureVideo   
    #ret
    #frameInit
    
#Methodes :
    #initialisation()
    #Read()
    #Show(frame)
    
    
class Camera:

    #Mise en place de la camera
    def __init__(self, id, cameraModel, mireParams):
        self.id = id
        self.cameraModel = cameraModel
        self.mire = mireParams
        self.Mint = np.matrix([[cameraModel.f.value / cameraModel.s1.value, 0, cameraModel.w.value/2], 
                              [0, cameraModel.f.value / cameraModel.s2.value, cameraModel.h.value/2],
                              [0,                    0,                       1]])
        self.idx = 1
        
    def initialisation(self):
        self.captureVideo = cv2.VideoCapture(self.id, cv2.CAP_DSHOW)
        
    def capture(self):
        ret, frame = self.captureVideo.read()
        return ret, frame
    
    #Calibration de la camera
    def calibrate(self, frame):
        ret, corners = cv2.findChessboardCorners(frame, (self.mire[0], self.mire[1]), cv2.CALIB_CB_FAST_CHECK)
        if (ret):
            cv2.drawChessboardCorners(frame, (self.mire[0], self.mire[1]), corners, ret)
            coord_px = corners.reshape(-1,2)
            flag = cv2.CALIB_USE_INTRINSIC_GUESS
            #retourne extr et change intr (mieux si plusieurs mires dans l espace)
            #ret, self.Mint, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera([self.mire[2]], [coord_px], (self.cameraModel.w.value, self.cameraModel.w.value), self.Mint, None, flags = flag)
            #retourne les params extr (mieux si une unique mire et intr connu)
            ret, self.rvecs, self.tvecs = cv2.solvePnP(self.mire[2], coord_px, self.Mint, None) 
        return ret, frame
    
    #(fonctionne mal)
    def undistortion(self, frame):
        self.Mint, roi = cv2.getOptimalNewCameraMatrix(self.Mint, self.dist, (1280, 960), 1, (1280, 960))
        frame = cv2.undistort(frame, self.Mint, self.dist, None, self.Mint)
        # crop the image
        x, y, w, h = roi
        if (x>0 & y>0):
            frame = frame[y:y+h, x:x+w]
        return frame
    
    def projectionPoints(self, coord_proj):
        #self.proj = cv2.projectPoints(coord_proj, self.rvecs[0], self.tvecs[0], self.Mint, None)[0].astype(int) #pour calibrateCamera
        self.proj = cv2.projectPoints(coord_proj, self.rvecs, self.tvecs, self.Mint, None)[0].astype(int) #pour solvePnP
        return self.proj

    #def showProjectionPoints(self)
        
    #Traitement d'image
    
    
    #Affichage    
    def show(self, frame, name):
        cv2.imshow('camera ' + str(self.id) + " : " + name, frame)
        

        
    #Destruction de la camera
    def release(self):
        self.captureVideo.release()
        
        
    
    