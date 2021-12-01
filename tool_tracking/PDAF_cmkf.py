# -*- coding: utf-8 -*-
"""
Created on Thu Aug 15 09:51:33 2019

@author: bpanneti
"""

 
 
from tool_tracking.motionModel import MotionModel, StateType, F, Q
from tool_tracking.estimator import Estimator, TRACKER_TYPE
#import tool_tracking as tr 

from point import Position
from orientation import Orientation
import numpy as np


class PDAF_cmkf(Estimator):# ENSTA  A modifier ici : changement de classe
    __metaclass__ = Estimator
    
    def __init__(self, parent=None, infos=None):
        super().__init__(parent, infos)
        self.type   = TRACKER_TYPE.CMKF_PDAF  
 

    def initializeTrack(self, plot):
        super().initializeTrack(plot)

        self.myTrack.initialize([plot], self.scan.dateTime, 0, 0, 0, [], [], [MotionModel.CV, 1])
        self.tracks.append(self.myTrack)
        
    def updateTrack(self, plot, unUpdatedTrack, track):
        super().updateTrack(plot, unUpdatedTrack, track)

        track.update([plot])
            
    @staticmethod
    def predictor(currState, time , flagChange):
        periode                        =  currState.time.msecsTo(time)/1000
     
        currState.xPred                =  F(periode, currState.state.shape[0], currState.filter[0])@currState.state #np.matrix(np.dot(F(periode, self.state.shape[0]), self.state))
        currState.pPred                =  F(periode, currState.state.shape[0], currState.filter[0])@currState.covariance@ F(periode, currState.state.shape[0],currState.filter[0]).T + Q(periode,currState.state.shape[0],currState.filter[0],currState.filter[1])
        currState.timeWithoutPlot     += periode

        if flagChange:
            currState.state = currState.xPred
            currState.covariance = currState.pPred

            currState.time = time

            currState.updateLocation()
            currState.updateCovariance() 
    @staticmethod
    def  estimator(plots, currState, posCapteur=Position(), orientationCapteur=Orientation()):
 
     
        print('-----> in pdaf estimator : non-parametric version')
        if plots==[]:
            return
    
        
        H = np.zeros([2,4])
        H[0,0] = 1
        H[1,2] = 1
            
        likelihood              = np.zeros((len(plots)+1,1))
        beta                    = np.zeros((len(plots)+1,1)) 
        state                   = np.zeros((currState.xPred.shape[0],len(plots)+1))
        covariance              = np.zeros((currState.xPred.shape[0],currState.xPred.shape[0],len(plots)+1))
  
        state[:,[0]]            = currState.xPred
        covariance[:,:,0]     = currState.pPred 
        
        #Valeur par défaut
        Pd      = 0.9
        Pg      = 0.9
        Volume  = 1000
        
        b  = (1 -Pd*Pg) / Pd * len(plots)/Volume
    
        print('-----> in pdaf estimator : number of validated measures {0}'.format(len(plots)))
        
        for ind,_plot in zip(range(0,len(plots)),plots) : 
            z                           = np.zeros([2,1])
            z[0]                        = _plot.z_XY[0]
            z[1]                        = _plot.z_XY[1]     
            R                           = np.zeros([2,2])
            R[0:2,0:2]                  = _plot.R_XY[0:2,0:2]
            
            
            In                          =   z - np.dot(H,currState.xPred) 
            S                           =   R + np.dot(H, np.dot(currState.pPred, H.T))
            K                           =   np.dot(currState.pPred, np.dot(H.T, np.linalg.inv(S)))
            likelihood[ind+1]           =   In.T@np.linalg.inv(S)@In
            
            state[:,[ind+1]]            =   currState.xPred+ K@In
            covariance[:,:,ind+1]       =   np.array(np.dot(np.dot(np.identity(currState.xPred.shape[0]) - np.dot(K,H), currState.pPred), (np.identity(currState.xPred.shape[0]) - np.dot(K, H)).T) + np.dot(K, np.dot(R, K.T))) 

        
        for ind in range(0,len(plots)+1):
            if ind == 0:
                pass
            else:
                pass

                #combinaison
        currState.state         = np.zeros((currState.state.shape[0],1))
        currState.covariance    = np.zeros((currState.state.shape[0],currState.state.shape[0]))
   
        for ind in range(0,len(plots)+1):
            pass
        for ind in range(0,len(plots)+1):
            pass
            pass
            
        currState.location.setXYZ(float(currState.state[0]),float(currState.state[2]), 0.0, 'ENU')
        currState.updateCovariance()    
            
     