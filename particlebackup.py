#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 16 21:55:02 2023

@author: hrb_xubu
"""




#

import numpy as np
from numpy.random import randn
from waypointShared import lineSensorResponse, lineDist
import random
import numpy.matlib
from scipy.stats import multivariate_normal
from joy import progress 

class partFilter:
    def __init__(self, num_particles, init_pos, M, Q):
        
        self.initpos = init_pos #seems to be the initaial position of the robot itself

        self.M = M # movement noise [ movenoice,movenouse, rotnoise]
        self.Q = Q # sensor noise [vale]
        
        # PF parameters
        self.mu = init_pos["mu"]              # variable dic
        self.Sigma = init_pos["Sigma"]
        self.num_particles = num_particles    
        self.particle = np.zeros((3, num_particles))       #particle position array
        self.particle_weights = (1/self.num_particles) * np.ones(num_particles)   #particle wt array creation with initial weights
        self.particle_sensor = np.zeros((2,num_particles))      #particle sensor array with 0 initial measurment
        for i in range(100):
            progress("Particle filter iniitalized")
        progress(self.mu)

        for i in range(self.num_particles):
            state = np.random.multivariate_normal(init_pos["mu"], init_pos["Sigma"]) #particle wt initialization
            self.particle[:,i] = state
            progress("__________")
        progress("__________")
        progress(self.particle)
        
      
            
    def justCheck(self):
        for i in range(100):
            progress("Particle filter check")
        
    def actionModel(self, dist):         #dist= [x,y,theta] distance traveled along X,Y and theta
        for i in range(self.num_particles):
            self.noise = np.dot(self.M, np.diag(np.square(dist))).flatten()   #dot product of noise constant and distance traveled alnd x,y,theta
            self.noise = self.noise.dot(np.array(randn(3,1)))     #dot profuct of self.noise with rand(3,1)
           
            self.particle[:,i] = self.particle[:,i]+(dist*self.noise) # adding  distance traveled and noise to each particle
            
            
    def sensorModel(self, lastwaypoint, nextwaypoint):
        # Get particle sensor measurements
        for i in range(self.num_particles):
            theta = self.particle[2,i]
            fpt = 1*self.particle[0,i] + self.particle[1,i]*1j - 10*np.exp(1j*theta)
            bpt = 1*self.particle[0,i] + self.particle[1,i]*1j - 10*np.exp(1j*theta)
            
            '''
            fd = lineDist(fpt,lastwaypoint, nextwaypoint)
            if np.isinf(fd):
                fs=0
            else:
                progress(fd)
                progress(self.Q)
                fs = lineSensorResponse(fd, self.Q)
            
            bd = lineDist(bpt,lastwaypoint, nextwaypoint)
            if np.isinf(bd):
                bs=0
            else:
            
                bs = lineSensorResponse(bd, self.Q)
            
            '''
            
            fd = lineDist(fpt,lastwaypoint, nextwaypoint)
            fs = lineSensorResponse(fd, self.Q)
            
            bd = lineDist(bpt,lastwaypoint, nextwaypoint)
            bs = lineSensorResponse(bd, self.Q)
            
            self.particle_sensor[:,i] = np.array([fs,bs])  #what will be the format of this?? going ahead assuming it is np.array(2x1)

    def correction(self, robotsensor):
        for i in range(self.num_particles):

            #wgt = wgt * 0.6 + 0.4 / (1.0+abs(spos-s)*0.5)

            '''
            error = abs(self.particle_sensor[:i] - robotsensor)
            
            if error[0]<50 and error[1]<50:
                self.particle_weights[i]=self.particle_weights[i]*2
            else:
                self.particle_weights[i]=self.particle_weights[i]*0.5
            '''
            innovation = self.particle_sensor[:,i] - robotsensor
            self.particle_weights[i]= self.particle_weights[i]*0.6 + 0.4/(1+np.linalg.norm(innovation)*0.5)

       # self.particle_weights = self.num_particles * self.particle_weights/np.sum(self.particle_weights)
        
        self.resample()        
        self.meanrevauation()
        
    def resample(self):
        best = np.argmax(self.particle_weights)
        best_weight = np.max(self.particle_weights)
        for i in range(self.num_particles):
            if self.particle_weights[i] < 0.25 * best_weight:
                self.particles[:,i] = self.particles[:, best]
                self.particle_weights[i] = 1.0
                
    def meanrevauation(self):
        
        self.mu = self.particle[:,np.argmax(self.particle_weights)]
        

        progress(int(self.mu[0]/100))
        progress(int(self.mu[1]/100))
        
        
        
        particle_mu = np.mean(self.particle,axis=1)
        progress("_____________")
        progress(particle_mu)
        
        sinSum = 0
        cosSum = 0
        
        for s in range(self.num_particles):
            cosSum = cosSum + np.cos(self.particle[2,s])
            sinSum = sinSum + np.sin(self.particle[2,s])
            
        particle_mu[2] = np.arctan2(sinSum,cosSum)
        
        zeroMean = self.particle - np.matlib.repmat(np.vstack(particle_mu),1,self.num_particles)
        
        self.Sigma = np.dot(zeroMean, np.transpose(zeroMean))/self.num_particles
        
   
            



        

    

        



    






