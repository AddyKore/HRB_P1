


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
        self.particle_mu=np.zeros((3,1))
        
        for i in range(100):
            progress("Particle filter initialization/re-initialization")
        

        for i in range(self.num_particles):
            state = np.random.multivariate_normal(init_pos["mu"], init_pos["Sigma"]) #particle wt initialization
            self.particle[:,i] = state

       
    def justCheck(self):
        for i in range(100):
            pass
        
    def actionModel(self, dist):         #dist= [x,y,theta] distance traveled along X,Y and theta
        
        
        for i in range(self.num_particles):
            #self.noise = np.dot(self.M, np.diag(np.square(dist))).flatten()   #dot product of noise constant and distance traveled alnd x,y,theta
            #self.noise = self.noise.dot(np.array(randn(3,1)))     #dot profuct of self.noise with rand(3,1)
            
            self.particle[:,i] = self.particle[:,i]+dist # adding  distance traveled and noise to each particle

        
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
                self.particle[:,i] = self.particle[:, best]
                self.particle_weights[i] = 1.0
                
    def meanrevauation(self):
        
        self.mu = self.particle[:,np.argmax(self.particle_weights)]
        
        progress("_____________")
        progress("_____________")
        progress(self.mu)
      

        
        
        self.particle_mu = np.mean(self.particle,axis=1)
        progress("_____________")

        progress(self.particle_mu)
        
        sinSum = 0
        cosSum = 0
        
        for s in range(self.num_particles):
            cosSum = cosSum + np.cos(self.particle[2,s])
            sinSum = sinSum + np.sin(self.particle[2,s])
            
        self.particle_mu[2] = np.arctan2(sinSum,cosSum)
        
        zeroMean = self.particle - np.matlib.repmat(np.vstack(self.particle_mu),1,self.num_particles)
        
        self.Sigma = np.dot(zeroMean, np.transpose(zeroMean))/self.num_particles
        
   
            

'''

        #issues faced---
        1) propogation was in 1000s - cause, I multiplied noise with distnce instead of adding it in action model
        2) Propogation was still in 100s, instead of 1s, thats cos the scale of propogation on the simulation screen and that on the
        praticles was not to the scale so had to divide it by 10 and 3 not really happy with this 
        
        Todo next:-
        1)update particle filter by reinitializing particles when the current waypoint is updated - Done
        2) Now you have to write a code to make the robot move using particle filter ok good o this tomorow -Done buttt - the particles are propogating before the bot can move solved it by using yield instead of start
        3) the particles are propogating to much so changed the devisor
    
    
    issues to discuss:- communication diagram, 
    how to solve the propogation issue and scalling of movements in sim and in code,
    doviz isualisation
'''

    

        



    





