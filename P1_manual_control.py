from joy import JoyApp
from joy.decl import *
from joy.plans import Plan
import time



#speed = 10

class movex( Plan ):
    def __init__(self,*arg,**kw):
        Plan.__init__(self, *arg, **kw)
        self.x1 = self.app.x1
        self.x2 = self.app.x2
        self.speed = self.app.speed
        self.x1.set_mode(2)
        self.x2.set_mode(2)


        self.x1.set_mode(2)
        self.x2.set_mode(2)

    def onStart( self ):
        progress("Starting driving")

    def onStop ( self ):
        progress("Stopping driving")
    
    def behavior(self):
        yield
        self.x1Pos=self.x1.get_pos()
        self.x2Pos=self.x2.get_pos()
        self.speed = self.app.speed

        self.x1.set_speed(self.speed)
        self.x2.set_speed(self.speed)

        progress(self.x2Pos)
        self.x1Pos= self.x1Pos+1000
        self.x2Pos= self.x2Pos+1000
        self.x1.set_pos(self.x1Pos%360000)   # motors need time to process the movement before we can send them next move
        self.x2.set_pos(self.x2Pos%360000)
        yield 15/self.speed
        
        #so if there is while loop it is stuck in there
        #is it is simple set pos then delay is needed
class movey( Plan ):
    def __init__(self,*arg,**kw):
        Plan.__init__(self, *arg, **kw)
        self.y1 = self.app.y1
        self.y2 = self.app.y2
        self.speed = self.app.speed


        self.y1.set_mode(2)
        self.y2.set_mode(2)

    def onStart( self ):
        progress("Starting driving")

    def onStop ( self ):
        progress("Stopping driving")
    
    def behavior(self):
        yield
       
        self.speed = self.app.speed

        self.y1.set_speed(self.speed)
        self.y2.set_speed(self.speed)

        self.y1Pos= self.y1.get_pos()
        self.y2Pos= self.y2.get_pos()
        progress("negative")
        self.y1Pos-=  1000
        self.y2Pos-= 1000
        self.y1.set_pos(self.y1Pos%360000)   # motors need time to process the movement before we can send them next move
        self.y2.set_pos(self.y2Pos%360000)
        yield 15/self.speed
        


class moveyn( Plan ):
    def __init__(self,*arg,**kw):
        Plan.__init__(self, *arg, **kw)
        self.y1 = self.app.y1
        self.y2 = self.app.y2
        self.speed = self.app.speed


        self.y1.set_mode(2)
        self.y2.set_mode(2)

    def onStart( self ):
        progress("Starting driving")

    def onStop ( self ):
        progress("Stopping driving")
    
    def behavior(self):
        yield
        
        self.speed = self.app.speed

        self.y11Pos= self.y1.get_pos()
        self.y22Pos= self.y2.get_pos()
        progress("positivve")
        self.y11Pos+= 1000
        self.y22Pos+= 1000
        self.y1.set_pos(self.y11Pos%360000)   # motors need time to process the movement before we can send them next move
        self.y2.set_pos(self.y22Pos%360000)
        yield 15/self.speed
        
class movexn( Plan ):
    def __init__(self,*arg,**kw):
        Plan.__init__(self, *arg, **kw)
        self.x1 = self.app.x1
        self.x2 = self.app.x2
        self.speed = self.app.speed


        self.x1.set_mode(2)
        self.x2.set_mode(2)

    def onStart( self ):
        progress("Starting driving")

    def onStop ( self ):
        progress("Stopping driving")
    
    def behavior(self):
        yield
        self.x1Pos=self.x1.get_pos()
        self.x2Pos=self.x2.get_pos()
        
        self.speed = self.app.speed

        self.x1.set_speed(self.speed)
        self.x2.set_speed(self.speed)

        progress(self.x2Pos)
        
        self.x1Pos= self.x1Pos-1000
        self.x2Pos= self.x2Pos-1000
        
        self.x1.set_pos(self.x1Pos%360000)   # motors need time to process the movement before we can send them next move
        self.x2.set_pos(self.x2Pos%360000)
        
        yield 15/self.speed


class botControl(JoyApp):
    def __init__(self,x1,x2,y1,y2,*arg,**kw):
        JoyApp.__init__(self, *arg,**kw)
        self.x1 = getattr(self.robot.at, x1)
        self.x2 = getattr(self.robot.at, x2)
        self.y1 = getattr(self.robot.at, y1)
        self.y2 = getattr(self.robot.at, y2)
        self.speed = 10
        
    
    def onStart(self):
        self.movex= movex(self)
        self.movexn= movexn(self)
        self.movey= movey(self)
        self.moveyn= moveyn(self)
        progress("Initializing...")
        #self.moveForward.start()
    


    

    def onEvent(self,evt):
        if evt.type != KEYDOWN:
            return
        # assertion: must be a KEYDOWN event

        if evt.key == K_1:
            self.speed = 5

        if evt.key == K_2:
            self.speed = 10

        if evt.key == K_3:
            self.speed = 15

        if evt.key == K_4:
            self.speed = 20
        
        if evt.key == K_UP and not self.movex.isRunning():
            self.movex.start()
            
        elif evt.key == K_DOWN and not self.movexn.isRunning():
            self.movexn.start()
            

        elif evt.key == K_RIGHT and not self.movey.isRunning():
            self.movey.start()
            
        elif evt.key == K_LEFT and not self.moveyn.isRunning():
            self.moveyn.start()
            

  
        else:
            return JoyApp.onEvent(self,evt)

    

    


if __name__ == "__main__":
    from sys import argv, stdout, exit

    robot = None          #setting variables for robot module count and module address
    x1 = "#x1"
    x2 = "#x2"
    y1 = "#y1"
    y2 = "#y2"

    args = list(argv[1:])

    while args:
        arg = args.pop(0)
        if arg=='--mod-count' or arg == '-c':
            #detects number of modules specified after -c
            N = int(args.pop(0))
            robot = dict(count=N)

        elif arg==' --x1' or arg=='-x1':
            #detects the address of the upper module
            x1 = args.pop(0)

        elif arg==' --x2' or arg=='-x2':
            #detects the address of the lower module
            x2 = args.pop(0)
            
        elif arg==' --y1' or arg=='-y1':
            #detects the address of the upper module
            y1 = args.pop(0)

        elif arg==' --y2' or arg=='-y2':
            #detects the address of the lower module
            y2 = args.pop(0)


        elif arg=='--help' or arg == '-h':
            #help prompt
            stdout.write(""" """%argv[0])
            exit(1)

    # ENDS cmdline parsing loop
  #start an interface

    app = botControl(x1,x2,y1,y2,robot=robot)
    app.run()
    