# file robotSimulator.py simulates a robot in an arena

from numpy import mean, abs, asarray
from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from plotIX import JoyAppWptSensorMixin
import numpy as np
import time
from partical import partFilter

from waypointShared import (
    WAYPOINT_HOST, WAYPOINT_MSG_PORT
    )

from pylab import randn,dot,mean,exp,newaxis



#move plans
class MoveForward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.movef(step)
      yield self.forDuration(dt)
      
      
      
class MoveBackward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.moveb(step)
      yield self.forDuration(dt)

class MoveLeft(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.movel(step)
      yield self.forDuration(dt)
      
      
class MoveRight(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.mover(step)
      yield self.forDuration(dt)


#move plans
class smallMoveForward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 1
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.movef(step)
      yield self.forDuration(dt)
      
      
      
class smallMoveBackward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 1
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.moveb(step)
      yield self.forDuration(dt)

class smallMoveLeft(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 1
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.movel(step)
      yield self.forDuration(dt)
      
      
class smallMoveRight(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.

  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 1
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.mover(step)
      yield self.forDuration(dt)



class MoveOnU(Plan):
    """
    Plan simulates robot moving by it self.

    (MODIFY THIS FOR YOUR ROBOT)
    """
    def __init__(self,app, moveF, moveB, moveL, moveR,pf,nxtwaypoint, meas_curr, tolerance =10):
      Plan.__init__(self,app)    
      self.moveF = moveF
      self.moveR = moveR
      self.moveL = moveL
      self.moveB = moveB
      self.pf=pf
      self.nxtwaypoint = nxtwaypoint
      self.meas_curr = meas_curr
      self.tolerance = tolerance
      self.flag = 0
      self.currWaypoint = np.zeros([0,0])
      self.particless=np.zeros((3,1))
      
    def updateParticles(self,dist):
        self.pf.justCheck()
        #self.pf.actionModel(dist)
        self.pf.actionModel(dist)
        lastwp = self.nxtwaypoint[0][0]+1j*self.nxtwaypoint[0][1]
        nxtwp = self.nxtwaypoint[1][0]+1j*self.nxtwaypoint[1][1]

        self.pf.sensorModel(lastwp, nxtwp)
        self.pf.correction(self.meas_curr)
        self.particless=self.pf.particle_mu
        
        progress(self.particless)
        progress("updated")
        progress(self.particless)
       
        #progress(self.pf.mu/100)
        

    def behavior(self):
        #progress(self.nxtwaypoint)
        #progress(self.meas_curr)
       # for i in range(100):
            #progress("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        if self.currWaypoint != self.nxtwaypoint[0]:
            num_particles = 100
            Q=0.01
            M=np.array([1,1,(np.pi/180*3)**2])
            x = self.nxtwaypoint[0][0]
            y = self.nxtwaypoint[0][1]
            init_state= np.array([x,y,0])
            #init_state= np.array([-1,-45,0])
            init_cov = np.diag([4,4, (np.pi/180*3)**2])
            init_pos = {"mu":init_state, "Sigma":init_cov}
                  
            self.pf = partFilter(num_particles, init_pos, M,Q)
            
            self.currWaypoint = self.nxtwaypoint[0]
            progress(self.pf.particle)
        
        if self.flag == 0:
            if((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])<0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])<0):# nxt target in +ve x and y
                if self.moveR.isRunning(): pass
                self.moveR.dist = 10.0
                #self.moveR.start()
                yield self.moveR
                #while self.moveR.isRunning(): pass

                self.updateParticles(np.array([self.moveR.dist/2.5,0,0]))
                
                
                if self.moveF.isRunning(): pass
                self.moveF.dist = 10.0
                #self.moveF.start()
                yield self.moveF
                #while self.moveF.isRunning(): pass

                self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                
                #progress("(say) North east")
                
            elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])>0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])>0):# nxt target in -ve x and y
                if self.moveL.isRunning(): pass
                self.moveL.dist = 10.0
                #self.moveL.start()
                yield self.moveL
                #while self.moveL.isRunning(): pass

                self.updateParticles(np.array([-1*self.moveL.dist/2.5,0,0]))
                
                if self.moveB.isRunning(): pass
                self.moveB.dist = 10.0
                #self.moveB.start()
                yield self.moveB
                #while self.moveB.isRunning(): pass

                self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                
                #progress("(say) South west")
                
            elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])<0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])>0):# nxt target in +ve x and -ve y
                if self.moveR.isRunning(): pass
                self.moveR.dist = 10.0
                #self.moveR.start()
                yield self.moveR
                #while self.moveR.isRunning(): pass

                self.updateParticles(np.array([self.moveR.dist/2.5,0,0]))
                
                if self.moveB.isRunning(): pass
                self.moveB.dist = 10.0
                #self.moveB.start()
                yield self.moveB
                #while self.moveB.isRunning(): pass
                
                self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                
                #progress("(say) South east")
                
            elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])>0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])<0):# nxt target in -ve x and y
                if self.moveL.isRunning(): pass
                self.moveL.dist = 10.0
                #self.moveL.start()
                yield self.moveL
                #while self.moveL.isRunning(): pass

                self.updateParticles(np.array([-1*self.moveL.dist/2.5,0,0]))
                
                if self.moveF.isRunning(): pass
                self.moveF.dist = 10.0
                #self.moveF.start()
                yield self.moveF
                #while self.moveF.isRunning(): pass

                self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                
                #progress("(say) North west")
            
            self.flag=0
            progress(self.pf.particle_mu)
            progress(self.nxtwaypoint[1])
           
            
            
        while(len(self.nxtwaypoint)>1):
            yield
            distx=min(50, abs(int(self.pf.particle_mu[0]-self.nxtwaypoint[1][0])))
            disty=  min(50, abs(int(self.pf.particle_mu[1]-self.nxtwaypoint[1][1])))
                       
            progress("((((((((((((((((((((()))))))))))))))))))))")
            for i in range(10):
                progress( distx)
                progress( disty)
                
        
            if self.pf.particle_mu[0] < self.nxtwaypoint[1][0] and self.pf.particle_mu[1] < self.nxtwaypoint[1][1]:
                
                    
                progress("(say) north east")
                if self.moveR.isRunning(): pass
                self.moveR.dist = distx
                #self.moveR.start()
                yield self.moveR
                #while self.moveR.isRunning(): pass
    
                self.updateParticles(np.array([self.moveR.dist/4,0,0]))
                
                
                if self.moveF.isRunning(): pass
                self.moveF.dist =  disty
                #self.moveF.start()
                yield self.moveF
               # while self.moveF.isRunning(): pass
    
                self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                
            elif self.pf.particle_mu[0] > self.nxtwaypoint[1][0] and self.pf.particle_mu[1] > self.nxtwaypoint[1][1]:
                progress("(say) south west")
                if self.moveL.isRunning(): pass
                self.moveL.dist =  distx
                #self.moveL.start()
                yield self.moveL
                #while self.moveL.isRunning(): pass
    
                self.updateParticles(np.array([-1*self.moveL.dist/2.5,0,0]))
                
                if self.moveB.isRunning(): pass
                self.moveB.dist = disty
                #self.moveB.start()
                yield self.moveB
                #while self.moveB.isRunning(): pass
    
                self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
            
            elif self.pf.particle_mu[0] < self.nxtwaypoint[1][0] and self.pf.particle_mu[1] > self.nxtwaypoint[1][1]:
                progress("(say) south east")
                if self.moveR.isRunning(): pass
                self.moveR.dist =  distx
                #self.moveR.start()
                yield self.moveR
                #while self.moveR.isRunning(): pass
    
                self.updateParticles(np.array([self.moveR.dist/4,0,0]))
                
                if self.moveB.isRunning(): pass
                self.moveB.dist =  disty
                #self.moveB.start()
                yield self.moveB
                #while self.moveB.isRunning(): pass
                
                self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                
            elif self.pf.particle_mu[0] > self.nxtwaypoint[1][0] and self.pf.particle_mu[1] < self.nxtwaypoint[1][1]:
                progress("(say)south west")
                if self.moveL.isRunning(): pass
                self.moveL.dist =  distx
                #self.moveL.start()
                yield self.moveL
                #while self.moveL.isRunning(): pass
    
                self.updateParticles(np.array([-1*self.moveL.dist/4,0,0]))
                
                if self.moveF.isRunning(): pass
                self.moveF.dist =  disty
                #self.moveF.start()
                yield self.moveF
                #while self.moveF.isRunning(): pass
    
                self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                
            elif self.pf.particle_mu[0] == self.nxtwaypoint[1][0] and self.pf.particle_mu[1] < self.nxtwaypoint[1][1]:
                progress("(say) move up")
                if self.moveF.isRunning(): pass
                self.moveF.dist =  distx
                #self.moveF.start()
                yield self.moveF
               # while self.moveF.isRunning(): pass
    
                self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                
            elif self.pf.particle_mu[0] == self.nxtwaypoint[1][0] and self.pf.particle_mu[1] > self.nxtwaypoint[1][1]:
                progress("(say)move down")
                if self.moveB.isRunning(): pass
                self.moveB.dist =  disty
                #self.moveB.start()
                yield self.moveB
                #while self.moveB.isRunning(): pass
                
                self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                
            elif self.pf.particle_mu[0] < self.nxtwaypoint[1][0] and self.pf.particle_mu[1] == self.nxtwaypoint[1][1]:
                progress("(say)move right")
                if self.moveR.isRunning(): pass
                self.moveR.dist =  distx
                #self.moveR.start()
                yield self.moveR
                #while self.moveR.isRunning(): pass
    
                self.updateParticles(np.array([self.moveR.dist/4,0,0]))
               
            
            elif self.pf.particle_mu[0] > self.nxtwaypoint[1][0] and self.pf.particle_mu[1] == self.nxtwaypoint[1][1]:
                progress("(say)  move left")
                if self.moveL.isRunning(): pass
                self.moveL.dist =  disty
                #self.moveL.start()'
                yield self.moveL
                #while self.moveL.isRunning(): pass
    
                self.updateParticles(np.array([-1*self.moveL.dist/4,0,0]))
                
            
        
        
        
        
        
        
        
        
        
        
        
        
        '''
        
        if(self.meas_curr[0]<100 or self.meas_curr[1]<100 ):
            #navigation when sensor readings are different
            
            while(self.meas_curr[0]<210 and self.meas_curr[1]<210):
                yield
                if((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])<0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])<0):# nxt target in +ve x and y
                    if self.moveR.isRunning(): pass
                    self.moveR.dist = 10.0
                    self.moveR.start()

                    self.updateParticles(np.array([self.moveR.dist/3,0,0]))
                    
                    
                    if self.moveF.isRunning(): pass
                    self.moveF.dist = 10.0
                    self.moveF.start()

                    self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                    
                    #progress("(say) North east")
                    
                elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])>0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])>0):# nxt target in -ve x and y
                    if self.moveL.isRunning(): pass
                    self.moveL.dist = 10.0
                    self.moveL.start()

                    self.updateParticles(np.array([-1*self.moveL.dist/3,0,0]))
                    
                    if self.moveB.isRunning(): pass
                    self.moveB.dist = 10.0
                    self.moveB.start()

                    self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                    
                    #progress("(say) South west")
                    
                elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])<0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])>0):# nxt target in +ve x and -ve y
                    if self.moveR.isRunning(): pass
                    self.moveR.dist = 10.0
                    self.moveR.start()

                    self.updateParticles(np.array([self.moveR.dist/3,0,0]))
                    
                    if self.moveB.isRunning(): pass
                    self.moveB.dist = 10.0
                    self.moveB.start()
                    self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                    
                    #progress("(say) South east")
                    
                elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])>0 and (self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])<0):# nxt target in -ve x and y
                    if self.moveL.isRunning(): pass
                    self.moveL.dist = 10.0
                    self.moveL.start()

                    self.updateParticles(np.array([-1*self.moveL.dist/3,0,0]))
                    
                    if self.moveF.isRunning(): pass
                    self.moveF.dist = 10.0
                    self.moveF.start()

                    self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                    
                    #progress("(say) North west")
                    
                    
            
       
        
            
            
        #navigation when both sensors have readings above 150
        if((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])<0): # to make the range of x from -150 to 150 to 0 to 300

            if(self.meas_curr[0]> 150 and self.meas_curr[1]>150):
                if self.moveR.isRunning(): return
                self.moveR.dist = 30.0
                self.moveR.start()
                
                self.updateParticles(np.array([self.moveR.dist/3,0,0]))
                return progress("(say) Move Right")
            else:
                if((self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])<0):
                    if self.moveF.isRunning(): return
                    self.moveF.dist = 30.0
                    self.moveF.start()

                    self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                    return progress("(say) Move forward")
                else:
                    if self.moveB.isRunning(): return
                    self.moveB.dist = 30.0
                    self.moveB.start()

                    self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                    return progress("(say) Move Backwards")
                
        elif((self.nxtwaypoint[0][0]+150)-(150+self.nxtwaypoint[1][0])>0):

            if(self.meas_curr[0] > 150 and self.meas_curr[1]>150):
                if self.moveL.isRunning(): return
                self.moveL.dist = 30.0
                self.moveL.start()

                self.updateParticles(np.array([-1*self.moveL.dist/3,0,0]))
                return progress("(say) Move Left")
            
            else:
                if((self.nxtwaypoint[0][1]+150)-(150+self.nxtwaypoint[1][1])>0):
                    if self.moveF.isRunning(): return
                    self.moveF.dist = 30.0
                    self.moveF.start()

                    self.updateParticles(np.array([0,self.moveF.dist/10,0]))
                    return progress("(say) Move forward")
                else:
                    if self.moveB.isRunning(): return
                    self.moveB.dist = 30.0
                    self.moveB.start()

                    self.updateParticles(np.array([0,-1*self.moveB.dist/10,0]))
                    return progress("(say) Move Backwards")

        '''
            
 
      

      

class RobotSimulatorApp( JoyApp, JoyAppWptSensorMixin ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.

     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,wphPort=WAYPOINT_MSG_PORT,*arg,**kw):
    """
    Initialize the simulator
    """
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    JoyAppWptSensorMixin.__init__(self, 
      host=wphAddr, wptPort=wphPort
    )
    # ADD pre-startup initialization here, if you need it

  def onStart( self ):
    """
    Sets up the JoyApp and configures the simulation
    """
    ### DO NOT MODIFY ------------------------------------------
    JoyAppWptSensorMixin.onStart(self)
    # Set up the sensor receiver plan
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/5.0)
    self.T0 = self.now
    ### MODIFY FROM HERE ------------------------------------------
    self.robSim = SimpleRobotSim(fn=None)
    self.moveF = MoveForward(self,self.robSim)
    self.moveB = MoveBackward(self,self.robSim)
    self.moveL = MoveLeft(self,self.robSim)
    self.moveR = MoveRight(self,self.robSim)
   
    self.movesF = smallMoveForward(self,self.robSim)
    self.movesB = smallMoveBackward(self,self.robSim)
    self.movesL = smallMoveLeft(self,self.robSim)
    self.movesR = smallMoveRight(self,self.robSim)
    self.pf=None
    self.nxtwaypoint=None
    self.curwaypoint=None
    self.meas_curr = None
    self.moveA= MoveOnU(self, self.moveF,self.moveB,self.moveL,self.moveR,self.pf, self.nxtwaypoint, np.array([0,0]), 10)
    self.particle =np.zeros((3,1))

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      #progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
      self.moveA.meas_curr=[f,b]
      
    else:
        
      progress( "Sensor: << no reading >>" )
      
    ts,w = self.sensor.lastWaypoints
    
    if ts:
      #progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
      self.moveA.nxtwaypoint = w
      self.nxtwaypoint = w
      
     
    else:
        progress( "Waypoint: << no reading >>" )

  def doVis(self):
    ## Example of visualization API
    sim = self.robSim                        #this has robots position
    tag = sim.zTag * sim.ang + sim.pos     #tag position check
    
    
    
    
    # Laser axis is based on tag and turret
    c = mean(tag) # center
    r = mean(abs(tag-c))
    
    ax = sim.ang/sim.tang*-1j
    
    # Visualize laser
    vl = c + asarray([0,ax*100*r])
    
    # Start a new visual in robot subplot
    self.visRobotClear()

    # plot command in robot subplot,
    #   '~' prefix changes coordinates using homography
    self.visRobot('~plot',
        [int(v) for v in vl.real],
        [int(v) for v in vl.imag],
        c='g')
    self.visRobot('grid',1) 
    
    
    if self.moveA.particless[0]:
        self.particle=self.moveA.particless
        progress("IN       DOVISIISSSSSSSSSSSSSSSSSS")
        progress(self.particle)

   
    
    # Start a new visual in arena subplot
    self.visArenaClear()
    x=[int(self.particle[0]-10), int(self.particle[0]),int( self.particle[0])]
    y=[int(self.particle[1]),int(self.particle[1]+10), int(self.particle[1])]
    progress(x)
    progress(y)
    self.visArena('~plot',x,y,c='g') # plot a green square, xformed
    
    
    
    
    # plot command in arena subplot,
    #   '~' prefix changes coordinates using homography
    self.visArena('~plot',
        [int(v) for v in vl.real],
        [int(v) for v in vl.imag],
        c='g',alpha=0.5)
    self.visArena('plot',[1000,2000,2000,1000],[1000,2000,1000,1000],c='r')
    
    # We can call any plot axis class methods, e.g. grid
    self.visArena('grid',1)  

  def on_K_p(self,evt):
    num_particles = 100
    Q=0.01
    M=np.array([1,1,(np.pi/180*3)**2])
    x = self.nxtwaypoint[0][0]
    y = self.nxtwaypoint[0][1]
    init_state= np.array([x,y,0])
    #init_state= np.array([-1,-45,0])
    init_cov = np.diag([4,4, (np.pi/180*3)**2])
    init_pos = {"mu":init_state, "Sigma":init_cov}
          
    self.moveA.pf = partFilter(num_particles, init_pos, M,Q)
    return progress(" ")
    
  def on_K_UP(self,evt):
    if self.moveF.isRunning(): return
    self.moveF.dist = 100.0
    self.moveF.start()
    return progress(" Move forward")

  def on_K_DOWN(self,evt):
    if self.moveB.isRunning(): return
    self.moveB.dist = 100.0
    self.moveB.start()
    return progress(" Move back")
  
  def on_K_LEFT(self,evt):
    if self.moveL.isRunning(): return
    self.moveL.dist = 100
    self.moveL.start()
    return progress(" Move left")

  def on_K_RIGHT(self,evt):
    if self.moveR.isRunning(): return
    self.moveR.dist =100
    self.moveR.start()
    return progress(" Move right")

  def on_K_u(self,evt):
    if self.moveF.isRunning(): return
    self.moveF.dist = 300.0
    self.moveF.start()
    return progress(" Move forward")

  def on_K_j(self,evt):
    if self.moveB.isRunning(): return
    self.moveB.dist = 300.0
    self.moveB.start()
    return progress(" Move back")
  
  def on_K_h(self,evt):
    if self.moveL.isRunning(): return
    self.moveL.dist = 300
    self.moveL.start()
    return progress(" Move left")

  def on_K_k(self,evt):
    if self.moveR.isRunning(): return
    self.moveR.dist =300
    self.moveR.start()
    return progress(" Move right")


  def on_K_w(self,evt):
    if self.movesF.isRunning(): return
    self.movesF.dist = 10.0
    self.movesF.start()
    return progress(" Move forward")

  def on_K_s(self,evt):
    if self.movesB.isRunning(): return
    self.movesB.dist = 10.0
    self.movesB.start()
    return progress(" Move back")
  
  def on_K_a(self,evt):
    if self.movesL.isRunning(): return
    self.movesL.dist = 10
    self.movesL.start()
    return progress(" Move left")

  def on_K_d(self,evt):
    if self.movesR.isRunning(): return
    self.movesR.dist =10
    self.movesR.start()
    return progress(" Move right")
    
    
        


  def on_K_o(self,evt):
    if self.moveA.isRunning(): return
    while(1):
        yield
        time.sleep(0.3)
        if self.moveA.isRunning(): pass
        else: self.moveA.start()
        
        #progress(" Auto mode")

  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      #progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      ### Run simulation code; for production code, doVis must run without
      #   using anything from .robSim, and can then call .emitTagMsg without
      #   any parameters
      self.robSim.refreshState() # refresh the simulation state
      self.doVis() # prepare visualization
      self.emitTagMsg(self.robSim.getTagList()) # emit the message for that
    return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  from sys import argv
  print("""
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer at ip and port given on commandline

  USAGE:
    %s
        Connect to default host and port
    %s <host>
        Connect to specified host on default port
    %s <host> <port>
        Connect to specified host on specified port
  """ % ((argv[0],)*3))
  import sys
  cfg = {'windowSize' : [160,120]}
  if len(argv)>2:
      app=RobotSimulatorApp(wphAddr=argv[1],wphPort=int(argv[2]),cfg=cfg)
  elif len(argv)==2:
      app=RobotSimulatorApp(wphAddr=argv[1],cfg=cfg)
  else:
      app=RobotSimulatorApp(cfg=cfg)
  app.run()

'''
if __name__=="__main__":
  from sys import argv
  print("""
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer at ip and port given on commandline

  USAGE:
    %s
        Connect to default host and port
    %s <host>
        Connect to specified host on default port
    %s <host> <port>
        Connect to specified host on specified port
  """ % ((argv[0],)*3))
  import sys
  # cfg = {'windowSize' : [160,120]}
  # if len(argv)>2:
  #     app=RobotSimulatorApp(wphAddr=argv[1],wphPort=int(argv[2]),cfg=cfg)
  # elif len(argv)==2:
  #     app=RobotSimulatorApp(wphAddr=argv[1],cfg=cfg)
  # else:
  #     app=RobotSimulatorApp(cfg=cfg)
  # app.run()



  robot = {'count':4,'names':{0x25:'m1', 0x02: 'm2', 0x37: 'm3', 0x58: 'm4'}, 'fillMissing':True,'required':[0x25,0x02,0x37,0x58]}
  if len(sys.argv)>1:
      address = sys.argv[1]
  else:
      address = WAYPOINT_HOST
  app=RobotSimulatorApp(wphAddr=argv[1], robot=robot, cfg={'windowSize' : [800,600]})
  app.run()
'''
