#Classe Camera

import cv2
import numpy as np
    
class Camera:

    
    #Mise en place de la camera
    def __init__(self, id, cameraModel, mireParams):
        self.id = id
        self.cameraModel = cameraModel
        self.Mint = np.matrix([[cameraModel.f.value / cameraModel.s1.value, 0, cameraModel.w.value/2], 
                              [0, cameraModel.f.value / cameraModel.s2.value, cameraModel.h.value/2],
                              [0,                    0,                       1]])
        
        
    def initialisation(self):
        self.captureVideo = cv2.VideoCapture(self.id, cv2.CAP_DSHOW)
        return self.captureVideo
        
    
    def capture(self):
        ret, frame = self.captureVideo.read()
        return ret, frame
    
    
    #Calibration de la camera
    def calibrate(self, frame, mire):
        ret, corners = cv2.findChessboardCorners(frame, (mire[0], mire[1]), cv2.CALIB_CB_FAST_CHECK)
        
        if (ret):   
            #cv2.drawChessboardCorners(frame, (mire[0], mire[1]), corners, ret)
            coord_px = corners.reshape(-1,2)
                
            ret, self.rvecs, self.tvecs = cv2.solvePnP(mire[2], coord_px, self.Mint, False) 
            self.rvecs33, _ = cv2.Rodrigues(self.rvecs)
            self.projMat = np.dot(self.Mint, np.block([self.rvecs33, self.tvecs]))
        return ret, frame
    
    
    def cameraPosition(self):
        R, _ = cv2.Rodrigues(self.rvecs)
        R = np.transpose(R)
        cameraPos = -np.matmul(R, self.tvecs)
        self.position = cameraPos
        return cameraPos
    
    
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
        
        
    
    