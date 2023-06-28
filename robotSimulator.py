# file robotSimulator.py simulates a robot in an arena

from numpy import mean, abs, asarray
from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from plotIX import JoyAppWptSensorMixin

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

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )

  def doVis(self):
    ## Example of visualization API
    sim = self.robSim
    tag = sim.zTag * sim.ang + sim.pos
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
    # Start a new visual in arena subplot
    self.visArenaClear()
    # plot command in arena subplot,
    #   '~' prefix changes coordinates using homography
    self.visArena('~plot',
        [int(v) for v in vl.real],
        [int(v) for v in vl.imag],
        c='g',alpha=0.5)
    # We can call any plot axis class methods, e.g. grid
    self.visArena('grid',1)  
  
  def on_K_UP(self,evt):
    if self.moveF.isRunning(): return
    self.moveF.dist = 100.0
    self.moveF.start()
    return progress("(say) Move forward")

  def on_K_DOWN(self,evt):
    if self.moveB.isRunning(): return
    self.moveB.dist = -100.0
    self.moveB.start()
    return progress("(say) Move back")
  
  def on_K_LEFT(self,evt):
    if self.moveL.isRunning(): return
    self.moveL.ang = -100
    self.moveL.start()
    return progress("(say) Turn left")

  def on_K_RIGHT(self,evt):
    if self.moveR.isRunning(): return
    self.moveR.ang =100
    self.moveR.start()
    return progress("(say) Turn right")

  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
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
