"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from json import dumps as json_dumps
from waypointShared import (
    WAYPOINT_HOST, WAYPOINT_MSG_PORT, APRIL_DATA_PORT
    )
from sensorPlanTCP import SensorPlanTCP

class PlotMixin( object ):
  """
  mixin class providing remote plotting operations through waypointTask.py
  """
  def __init__(self):
    # Buffer for visualization requests
    self.visArenaClear()
    self.visRobotClear()

  def visArenaClear(self):
      """
      Reset / clear the arena visualization requests
      """
      self._lw = []

  def visRobotClear(self):
      """
      Reset / clear the robot visualization requests
      """
      self._lr = []

  def visArena(self,meth,*arg,**kw):
      """
      Add a visualization request to be plotted in the arena subplot
      INPUT:
        meth - str - axis method name
        *arg,**kw - additional arguments as appropriate for the visualilzation method

      NOTE: coordinates should be given in arena coordinates,
        i.e. with respect to the true world frame used to give
        the reference locations of the markers

      Example:
      >>> ix.visArenaClear()
      >>> ix.visArena('plot',x=[10,20,20,10,10],y=[10,10,20,20,10],c='r') # plot a red square
      """
      msg = { '@w' : meth }
      for n,v in enumerate(arg):
          msg['@%d' % n] = v
      msg.update(kw)
      self._lw.append(msg)

  def visRobot(self,meth,*arg,**kw):
      """
      Add a visualization request to be plotted in the robot subplot
      INPUT:
        meth - str - axis method name
        *arg,**kw - additional arguments as appropriate for the visualilzation method

      NOTE: coordinates should be given in arena coordinates,
        i.e. with respect to the true world frame used to give
        the reference locations of the markers
      """
      msg = { '@r' : meth }
      for n,v in enumerate(arg):
          msg['@%d' % n] = v
      msg.update(kw)
      self._lr.append(msg)

  def getPlotMsg( self ):
      return self._lw + self._lr
        
class JoyAppWptSensorMixin( PlotMixin ):
  def __init__(self, host=WAYPOINT_HOST, wptPort=WAYPOINT_MSG_PORT, tagPort=APRIL_DATA_PORT):
    PlotMixin.__init__(self)
    self.wptSvr = host
    self.wptPort = wptPort
    self.tagPort = tagPort
    self.wptSock = None
  
  def onStart(self):
    self.sensor = SensorPlanTCP(self,server=self.wptSvr,port=self.wptPort)
    self.sensor.start()

  def emitTagMsg(self,msg=()):
    s = self.wptSock
    if s is None: 
        s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
        s.bind(("",0))
    raw = json_dumps(list(msg)+self.getPlotMsg()).encode("ascii")
    try:
        s.sendto(raw, (self.wptSvr, self.tagPort))
    except SocketError as se:
        progress("SOCKET ERROR: %s" % se)
        try:
          s.close()
        finally:
          pass
        s = None
    self.wptSock = s
    
    
if __name__=="__main__":
    print("""
    Demonstrating plotting via waypointTask
    
    NOTE: typically JoyAppWptSensorMixin is a mixin for JoyApp-s, but it can be used this way
      provided a sensorPlan connects to the waypointTask to make it live
    """)

    DEFAULT_MSG_TEMPLATE = {
        0 : [[2016, 1070], [1993, 1091], [2022, 1115], [2044, 1093]],
        1 : [[1822, 1323], [1824, 1287], [1787, 1281], [1784, 1315]],
        2 : [[1795, 911], [1766, 894], [1749, 916], [1779, 933]],
        3 : [[1451, 876], [1428, 896], [1454, 917], [1476, 896]],
        4 : [[1374, 1278], [1410, 1268], [1399, 1236], [1364, 1243]],
        22 : [[1744, 622], [1743, 646], [1774, 650], [1774, 626]],
        23 : [[2274, 1171], [2312, 1177], [2306, 1146], [2271, 1141]],
        24 : [[1100, 975], [1110, 946], [1077, 938], [1066, 966]],
        25 : [[1666, 1629], [1665, 1589], [1625, 1585], [1626, 1624]],
        26 : [[2305, 1663], [2310, 1704], [2352, 1708], [2345, 1667]],
        27 : [[2230, 697], [2230, 721], [2262, 727], [2260, 704]],
        28 : [[911, 1525], [952, 1523], [953, 1483], [913, 1486]],
        29 : [[1222, 542], [1193, 537], [1186, 558], [1216, 566]],
    }
    def tags2list( dic ):
        """
        Convert a dictionary of tags into part of a list for JSON serialization

        INPUT:
          dic -- dictionary mapping tag id to 4x2 corner location array

        OUTPUT:
          list to be concatenated into JSON message
        """
        return [
        {
          'i' : k,
          'p': [ list(row) for row in v ]
        }
        for k,v in dic.items()
        ]
    from time import sleep
    wtm = JoyAppWptSensorMixin()
    while True:
        wtm.visArenaClear()
        x,y = [1000,2000,2000,1000,1000],[1000,1000,2000,2000,1000]
        wtm.visArena('plot',x,y,c='r') # plot a red square
        wtm.visArena('~plot',x,y,c='g') # plot a green square, xformed
        wtm.visRobotClear()
        wtm.visRobot('plot',x,y,c='b-') # plot a blue square
        wtm.visRobot('~plot',x,y,c='m') # plot a magenta square, xformed
        wtm.emitTagMsg(tags2list(DEFAULT_MSG_TEMPLATE))
        sleep(0.5)
          

