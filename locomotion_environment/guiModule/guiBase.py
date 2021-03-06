import wx
import sys
import os
import pydart2 as pydart
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import cMat 
import SimbiconController as SC

from wx import glcanvas

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class GuiBase(glcanvas.GLCanvas):
    def __init__(self, parent, sim, controller,env):
        glcanvas.GLCanvas.__init__(self, parent, -1)
        self.init = False

        self.sim = sim
        self.controller = controller
        self.env = env
        #TODO::initial Mouse Postition

        #
        self.size = None
        self.context = glcanvas.GLContext(self)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnMouseUp)
        self.Bind(wx.EVT_RIGHT_DOWN, self.OnRmouseDown)
        self.Bind(wx.EVT_RIGHT_UP, self.OnRmouseUp)

        self.timer = wx.Timer(self,id = 1)
        self.Bind(wx.EVT_TIMER, self.TimerEvent)
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_MOUSEWHEEL, self.OnMouseWheel)
        self.Bind(wx.EVT_MOTION, self.mouseMotion)
        self.timer.Start(30)
        self.timerOffset = 30
        self.currentTimeStep = 0
        #print(self.timer.GetInterval())
        #input()
        
        self.mouseDownPos = None
        self.mouseUpPos = None


    def OnEraseBackground(self,event):
        pass

    def OnSize(self, event):
        size = self.size = self.GetClientSize()

        if self.init:
            self.SetCurrent(self.context)
            glViewport(0,0, size.width, size.height)
        event.Skip()

    def OnPaint(self, event):
        self.SetCurrent(self.context)
        if not self.init:
            self.InitGL()
            self.init = True
        self.OnDraw()

    def OnMouseDown(self, event):
        return

    def OnMouseUp(self,event):
        return

    def OnRmouseDown(self, event):
        return

    def OnRmouseUp(self, event):
        return
    
    def OnMouseWheel(self, event):
        return

    def TimerEvent(self, event):
        self.TimerFunc()
    
    def TimerFunc(self):
        print("parents...")
        self.Refresh()
        return


skel_path = "/home/qfei/dart/data/sdf/atlas/"

if __name__ == '__main__':
    pydart.init()

    world = pydart.World(1/1000)


    ground = world.add_skeleton(skel_path+"ground.urdf")
    atlas = world.add_skeleton(skel_path+"atlas_v3_no_head_soft_feet.sdf");

    skel = world.skeletons[1]

    q = skel.q
    q[0] = -0.5*np.pi
    q[4] = q[4]+0.01
    skel.set_positions(q)

    controller = SC.Controller(skel,world)

    app = wx.App(0)

    frame = wx.Frame(None, -1, size=(1200,960))
    #gui = dartGui(frame,world,controller)
    gui = GuiBase(frame,world,controller)
    frame.Show(True)

    app.MainLoop()
