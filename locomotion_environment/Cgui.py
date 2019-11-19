import wx
import sys
import pydart2 as pydart
import numpy as np
import cMat
import SimbiconController as SC

from wx import glcanvas

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

import logging;logger = logging.getLogger("pyassimp_opengl")
logging.basicConfig(level= logging.INFO)

class GuiBase(glcanvas.GLCanvas):
    def __init__(self, parent, sim, controller):
        glcanvas.GLCanvas.__init__(self, parent, -1)
        self.init = False

        self.sim = sim
        self.controller = controller

        #TODO::initial Mouse Postition

        #
        self.size = None
        self.context = glcanvas.GLContext(self)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnMouseUP)

        self.timer = wx.Timer(self, 10)
        self.Bind(wx.EVT_TIMER, self.TimerEvent)

        self.timer.Start(10)
        

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

    def OnMouseUP(self,event):
        return

    def TimerEvent(self, event):
        self.TimerFunc()
    
    def TimerFunc(self):
        print("parents...")
        self.Refresh()
        return


class dartGuiNew(GuiBase):
    def InitGL(self):
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        #self.loadModel()

        glClearColor(1,1,1,1.)

        glEnable(GL_LIGHTING)

        glEnable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)

        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)
        glEnable(GL_NORMALIZE)
        glEnable(GL_LIGHT0)

        #glutDisplayFunc(###)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        #gluPerspective(3##)
        gluPerspective(60, 1, -3, 20)
        # position viewer
        glMatrixMode(GL_MODELVIEW)

        #setdefaultCamera
        glTranslatef(0, 0, -1.0)
        gluLookAt(10.0,10.0,10.0,0,0,0,0,1,0)


        
        #input(self.sim.skeletons[0].filename)
        #input(self.sim.skeletons[1].filename)

        #self.load_Model(self.sim.skeletons[0].filename)
        #self.load_Model(self.sim.skeletons[1].filename)
        #input()
        
        """
        glPushMatrix()
        input()
        glutMainLoop()
        """

        self.scene = None

    def OnDraw(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        self.drawSkeletons()
        self.SwapBuffers()
        

    def drawSkeletons(self,):
        for i in range(len(self.sim.skeletons)):
            self.drawSkeleton(self.sim.skeletons[i])
        
    def drawSkeleton(self, skeleton):
        self.drawBodyNode(skeleton.root_bodynode(0))

    def drawBodyNode(self, root_node):
        #glPushMatrix()
        print("done")
        #glMultMatrix()
        #print(root_node.relative_transform())
        #input()
        glMultMatrixd(root_node.relative_transform())
        
        for sf in root_node.shapenodes:
            self.drawShapeFrame(sf)

        recursive = True
        if(recursive):
            print(root_node.num_child_bodynodes())
            for i in range(root_node.num_child_bodynodes()):
                print("root_node's child:",i,root_node.child_bodynodes[i])
                self.drawEntity(root_node.child_bodynodes[i])

        #input()
        #glPopMatrix()


    def drawEntity(self, entity):
        print("drawEntity")
        if (entity is None):
            return;
        
        self.drawBodyNode(entity)

    def drawShapeFrame(self, shape_frame):
        print(shape_frame.visual_aspect_rgba())

        #glPushMatrix()
        glMultMatrixd(shape_frame.relative_transform())

        self.drawShape(shape_frame.shape, shape_frame.visual_aspect_rgba())

    def drawShape(self, shape, color):
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

        if (color is not None):
            glColor4dv(color)
        else:
            glColor4d(1,1,1,1)
        #glColor4d(color[0], color[1], color[2], color[3])
        #print(shape.shape_type_name())
        if(type(shape) is pydart.shape.BoxShape):
            self.drawCube(shape.size())
        elif (type(shape) is pydart.shape.SphereShape):
            self.drawSphere(shape.getRaius())
        elif (type(shape) is pydart.shape.MeshShape):
            glDisable(GL_COLOR_MATERIAL)
            print(shape)
            print(shape.displaylist())
            #input(shape.path())
            if(shape.displaylist() is not 0):
                self.drawList(shape.displaylist())
            else:
                #input(shape.path())
                mesh = self.loadMesh(shape.path())
                #input("Next Draw")
                #self.drawMesh(mesh)
            #getDisplayList
            #none..
            #self.drawMesh(MeshShape.scale)
        else:
            input(shape.shape_type_name())
        #input()

    def loadMesh(self, path, postprocess = None):
        logger.info("Loading Model:" + path + "...")

        if postprocess:
            self.scene = pyassimp.load(path, processing=postprocess)
        else:
            self.scene = pyassimp.load(path)
        
        logger.info("Done.")

        scene = self.scene
        #log some statistics
        logger.info(" meshes: %d" % len(scene.meshes))
        logger.info(" total faces: %d" % sum([len(mesh.faces) for mesh in scene.meshes]))
        logger.info(" metarials: %d" % len(scene.materials))
            
        for index, mesh in enumerate(scene.meshes):
            self.prepare_gl_buffer(mesh)
            self.drawMesh(mesh)

        pyassimp.release(scene)

    def drawMesh(self, mesh):
        #glPushMatrix()
        #OpenGL row major
        #print(mesh.faces)
        #input()
        self.apply_material(mesh.material)

        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["vertices"])
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, None)

        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["normals"])
        glEnableClientState(GL_NORMAL_ARRAY)
        glNormalPointer(GL_FLOAT, 0, None)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["triangles"])
        glDrawElements(GL_TRIANGLES, len(mesh.faces) * 3, GL_UNSIGNED_INT, None)

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, 0 )
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        #glPopMatrix()


    def apply_material(self, mat):
        if not hasattr(mat, "gl_mat"):
            diffuse = numpy.array(mat.properties.get("diffuse", [0.8, 0.8, 0.8, 1.0]))
            specular = numpy.array(mat.properties.get("specular", [0., 0., 0., 1.0]))
            ambient = numpy.array(mat.properties.get("ambient", [0.2, 0.2, 0.2, 1.0]))
            emissive = numpy.array(mat.properties.get("emissive", [0., 0., 0., 1.0]))   
            shininess = min(mat.properties.get("shininess", 1.0), 128)   
            wireframe = mat.properties.get("wireframe", 0)   
            twosided = mat.properties.get("twosided", 1)   
    
            setattr(mat, "gl_mat", glGenLists(1))   
            glNewList(mat.gl_mat, GL_COMPILE)

            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
            glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)
    
            glEndList()
    
        glCallList(mat.gl_mat)

  
    def load_Model(self, path, postprocess = None):
        logger.info("Loading Model:" + path + "...")

        if postprocess:
            self.scene = pyassimp.load(path, processing=postprocess)
        else:
            self.scene = pyassimp.load(path)
        
        logger.info("Done.")

        scene = self.scene
        #log some statistics
        logger.info(" meshes: %d" % len(scene.meshes))
        logger.info(" total faces: %d" % sum([len(mesh.faces) for mesh in scene.meshes]))
        logger.info(" metarials: %d" % len(scene.materials))
            
        for index, mesh in enumerate(scene.meshes):
            self.prepare_gl_buffer(mesh)

        pyassimp.release(scene)

    def prepare_gl_buffer(self, mesh):

        mesh.gl={}
        print(mesh.vertices)
        #input("meshes")
        mesh.gl["vertices"] = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["vertices"])
        glBufferData(GL_ARRAY_BUFFER, mesh.vertices, GL_STATIC_DRAW)

        mesh.gl["normals"] = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["normals"])
        glBufferData(GL_ARRAY_BUFFER, mesh.normals, GL_STATIC_DRAW)

        mesh.gl["triangles"] = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["triangles"])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.faces, GL_STATIC_DRAW)


        print(mesh.gl)
        #unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)


    
    def drawCube(self,_size):
        glScaled(_size[0], _size[1], _size[2])

        _n = np.array(
                [[-1., 0., 0.],
                [0., 1., 0.],
                [1., 0., 0.],
                [0., -1., 0.],
                [0., 0., 1.],
                [0., 0., -1.]])

        faces = np.array(
                [[0,1,2,3],
                [3,2,6,7],
                [7,6,5,4],
                [4,5,1,0],
                [5,6,2,1],
                [7,4,0,3]]
                )
        v = np.zeros((8,3))

        size = 1
        v[0:4][0] = -size/2
        v[4:8][0] = size/2
        v[0:2][1] = -size/2
        v[4:6][1] = -size/2
        v[2:4][1] = size/2
        v[6:8][1] = size/2
        v[0][2] = -size/2
        v[3][2] = -size/2
        v[4][2] = -size/2
        v[7][2] = -size/2
        v[1][2] = size/2
        v[2][2] = size/2
        v[5][2] = size/2
        v[6][2] = size/2

        i = 5
        print(_n[i][0])
        #input()
        while(i>=0):
            glBegin(GL_QUADS)
            glNormal3fv(_n[i])
            glVertex3fv(v[faces[i][0]])
            glVertex3fv(v[faces[i][1]])
            glVertex3fv(v[faces[i][2]])
            glVertex3fv(v[faces[i][3]])
            glEnd()
            i-=1
 

class dartGui(GuiBase):
    def InitGL(self):
        
        # set viewing projection
        glMatrixMode(GL_PROJECTION)
        #glFrustum(-0.5, 0.5, -0.5, 0.5, 1.0, 3.0)
        gluPerspective(60, 1, 1, 20)

        # position viewer
        glMatrixMode(GL_MODELVIEW)
        glTranslatef(0, 0, -1.0)
        gluLookAt(1.0,0.0,3.5,1.5,0,0,0,1,0)
        # position object
        #glRotatef(self.y, 1.0, 0.0, 0.0)
        #glRotatef(self.x, 0.0, 1.0, 0.0)

        glEnable(GL_DEPTH_TEST)
        #glEnable(GL_LIGHTING)
        #glEnable(GL_LIGHT0)
        #glutTimerFunc(1000, self.Timer, 0)
        


    def OnDraw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


        """
        bodyGround = self.sim.skeletons[0]
        rootnode = bodyGround.root_bodynodes()
        for i in rootnode:
            self.drawBody(i)
        #self.SwapBuffers()
        
        bodySkel = self.sim.skeletons[1]

        rootnode = bodySkel.root_bodynodes()
        #self.drawingBox(exambodynode[0].child_bodynodes[0])

        for i in rootnode:
            self.drawBody(i)
        self.SwapBuffers()
        """
        self.drawSkeletons()

        self.SwapBuffers()

    def drawBody(self,root):
        
        self.drawingBox(root)

        for i in root.child_bodynodes:
            self.drawBody(i)

    def drawingBox(self,body):
        shapelen = body.num_shapenodes()
        _trans = body.transform()

        for i in body.shapenodes:
            _shape = i.shape
            s_min = cMat.Matrix.multTrans(_trans,_shape.bounding_box()[0])
            s_max = cMat.Matrix.multTrans(_trans,_shape.bounding_box()[1])

            x = s_min[0]
            y = s_min[1]
            z = s_min[2]

            p = s_max[0]
            q = s_max[1]
            r = s_max[2]

            v1 = s_min
            v2 = (p,y,z)
            v3 = (x,q,z)
            v4 = (p,q,z)

            v5 = (x,y,r)
            v6 = (p,y,r)
            v7 = (x,q,r)
            v8 = s_max

            glBegin(GL_QUADS)
            
            glNormal3fv(cMat.Matrix.getNormal(v1,v2,v3))
            glVertex3fv(v1)
            glVertex3fv(v2)
            glVertex3fv(v3)
            glVertex3fv(v4)

            glNormal3fv(cMat.Matrix.getNormal(v1,v3,v7)) 
            glVertex3fv(v1)
            glVertex3fv(v3)
            glVertex3fv(v7)
            glVertex3fv(v5)

            glNormal3fv(cMat.Matrix.getNormal(v1,v2,v6))
            glVertex3fv(v1)
            glVertex3fv(v2)
            glVertex3fv(v6)
            glVertex3fv(v5)

            glNormal3fv(cMat.Matrix.getNormal(v2,v4,v8))
            glVertex3fv(v2)
            glVertex3fv(v4)
            glVertex3fv(v8)
            glVertex3fv(v6)

            glNormal3fv(cMat.Matrix.getNormal(v3,v4,v8))
            glVertex3fv(v3)
            glVertex3fv(v4)
            glVertex3fv(v8)
            glVertex3fv(v7)

            glNormal3fv(cMat.Matrix.getNormal(v5,v6,v8))
            glVertex3fv(v5)
            glVertex3fv(v6)
            glVertex3fv(v8)
            glVertex3fv(v7)

            glEnd()


    def TimerFunc(self,):
        #print("TImer....")
        #self.controller.update()
        #self.sim.step()

        #glutPostRedisplay()
        #self.OnDraw()
        self.Refresh()

        return

    def drawSkeletons(self,):
        for i in range(len(self.sim.skeletons)):
            self.drawSkeleton(self.sim.skeletons[i])
        
    def drawSkeleton(self, skeleton):
        self.drawBodyNode(skeleton.root_bodynode(0))

    def drawBodyNode(self, root_node):
        #glPushMatrix()
        print("done")
        #glMultMatrix()
        #print(root_node.relative_transform())
        #input()
        glMultMatrixd(root_node.relative_transform())
        
        for sf in root_node.shapenodes:
            self.drawShapeFrame(sf)

        recursive = True
        if(recursive):
            print(root_node.num_child_bodynodes())
            for i in range(root_node.num_child_bodynodes()):
                print("root_node's child:",i,root_node.child_bodynodes[i])
                self.drawEntity(root_node.child_bodynodes[i])

        #input()
        #glPopMatrix()


    def drawEntity(self, entity):
        print("drawEntity")
        if (entity is None):
            return;
        
        self.drawBodyNode(entity)

    def drawShapeFrame(self, shape_frame):
        print(shape_frame.visual_aspect_rgba())

        #glPushMatrix()
        glMultMatrixd(shape_frame.relative_transform())

        self.drawShape(shape_frame.shape, shape_frame.visual_aspect_rgba())

    def drawShape(self, shape, color):
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

        if (color is not None):
            glColor4dv(color)
        else:
            glColor4d(1,1,1,1)
        #glColor4d(color[0], color[1], color[2], color[3])
        #print(shape.shape_type_name())
        if(type(shape) is pydart.shape.BoxShape):
            self.drawCube(shape.size())
        elif (type(shape) is pydart.shape.SphereShape):
            self.drawSphere(shape.getRaius())
        elif (type(shape) is pydart.shape.MeshShape):
            glDisable(GL_COLOR_MATERIAL)
            print(shape)
            print(shape.displaylist())
            #input(shape.path())
            if(shape.displaylist() is not 0):
                self.drawList(shape.displaylist())
            else:
                #input(shape.path())
                self.loadMesh(shape.path())
            #getDisplayList
            #none..
            #self.drawMesh(MeshShape.scale)
        else:
            input(shape.shape_type_name())
        #input()

    
    def loadMesh(self, path, postprocess = None):
        logger.info("Loading Model:" + path + "...")

        if postprocess:
            self.scene = pyassimp.load(path, processing=postprocess)
        else:
            self.scene = pyassimp.load(path)
        
        logger.info("Done.")

        scene = self.scene
        #log some statistics
        logger.info(" meshes: %d" % len(scene.meshes))
        logger.info(" total faces: %d" % sum([len(mesh.faces) for mesh in scene.meshes]))
        logger.info(" metarials: %d" % len(scene.materials))
            
        for index, mesh in enumerate(scene.meshes):
            self.prepare_gl_buffer(mesh)

        pyassimp.release(scene)

    def prepare_gl_buffer(self, mesh):

        mesh.gl={}

        mesh.gl["vertices"] = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["vertices"])
        glBufferData(GL_ARRAY_BUFFER, mesh.vertices, GL_STATIC_DRAW)

        mesh.gl["normals"] = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["normals"])
        glBufferData(GL_ARRAY_BUFFER, mesh.normals, GL_STATIC_DRAW)

        mesh.gl["triangles"] = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["triangles"])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.faces, GL_STATIC_DRAW)


        print(mesh.gl)
        #unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)


    def drawCube(self,_size):
        glScaled(_size[0], _size[1], _size[2])

        _n = np.array(
                [[-1., 0., 0.],
                [0., 1., 0.],
                [1., 0., 0.],
                [0., -1., 0.],
                [0., 0., 1.],
                [0., 0., -1.]])

        faces = np.array(
                [[0,1,2,3],
                [3,2,6,7],
                [7,6,5,4],
                [4,5,1,0],
                [5,6,2,1],
                [7,4,0,3]]
                )
        v = np.zeros((8,3))

        size = 1
        v[0:4][0] = -size/2
        v[4:8][0] = size/2
        v[0:2][1] = -size/2
        v[4:6][1] = -size/2
        v[2:4][1] = size/2
        v[6:8][1] = size/2
        v[0][2] = -size/2
        v[3][2] = -size/2
        v[4][2] = -size/2
        v[7][2] = -size/2
        v[1][2] = size/2
        v[2][2] = size/2
        v[5][2] = size/2
        v[6][2] = size/2

        i = 5
        print(_n[i][0])
        #input()
        while(i>=0):
            glBegin(GL_QUADS)
            glNormal3fv(_n[i])
            glVertex3fv(v[faces[i][0]])
            glVertex3fv(v[faces[i][1]])
            glVertex3fv(v[faces[i][2]])
            glVertex3fv(v[faces[i][3]])
            glEnd()
            i-=1
    
    def drawSphere(self, radius):
        input("drawSphere")
    def drawEllipsoid(self, size):
        input("drawEllipsoid")
    def drawList(self, index):
        input("useDisplaylist")

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
    gui = dartGuiNew(frame,world,controller)
    frame.Show(True)

    app.MainLoop()
