#Classe Cameras

import cv2
import numpy as np

class Cameras :
    
    
    def __init__ (self, mireParams, cameras = []):
        self.nombre = len(cameras)
        self.list = cameras
        self.mire = mireParams
        
        
    def initialisation(self): 
        self.capturesVideos = []
        for camera in self.list :
            self.capturesVideos.append(camera.initialisation())
        
        
    def capture(self): 
        rets = []
        frames = []
        
        for camera in self.list :
            ret, frame = camera.captureVideo.read()
            rets.append(ret)
            frames.append(frame)
            
        return rets, frames
        
    
    def calibrate(self, framesIn):
        rets = []
        framesOut = []
        
        #FindChessBoardCorners
        for idx in range(self.nombre):
            ret, frame = self.list[idx].calibrate(framesIn[idx], self.mire)
            rets.append(ret)
            framesOut.append(frame)
        
        return rets, framesOut
        
    def camerasPos(self):
        positions = []
        for camera in self.list:
            positions.append(camera.cameraPosition())
        return positions
    
    
    def projectionPoints(self, coordProj):
        projections = []
        for camera in self.list:
            projections.append(camera.projectionPoints(coordProj))
        return projections
            
    def release(self):
        for camera in self.list:
            camera.release()
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            