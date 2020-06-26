
import wx
import sys
import os
import pydart2 as pydart
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import cMat 
import SimbiconController_3d as SC

from wx import glcanvas

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *


import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

import logging;logger = logging.getLogger("pyassimp_opengl")
logging.basicConfig(level= logging.INFO)

###
# Mesh Shape = 0
# Box Shape = 1
###

class drawingMesh():

    def __init__(self,shape,size = 0):
        self.scene = None
        self.whatShape = shape
        self.boxSize = size
    def getShape(self):
        return self.whatShape

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
        #print(scene.meshes[0])
        #input("before pgb")
        for index, mesh in enumerate(scene.meshes):
            self.prepare_gl_buffer(mesh)

        #print(scene.meshes[0])

        pyassimp.release(scene)

        #print(self.scene.meshes[0])
        #print(scene.meshes[0])

        #input("howaboutscene")

    def prepare_gl_buffer(self, mesh):

        mesh.gl={}
        #print(mesh.vertices)
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


        #print(mesh.gl)
        #unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        diffuse = numpy.array(mesh.material.properties.get("diffuse", [0, 1, 0.8, 0.3]))
        specular = numpy.array(mesh.material.properties.get("specular", [0., 0., 0., 0.3]))
        ambient = numpy.array(mesh.material.properties.get("ambient", [0, 1, 0.2, 0.3]))
        emissive = numpy.array(mesh.material.properties.get("emissive", [0., 0., 0., 0.3]))   
        shininess = min(mesh.material.properties.get("shininess", 1.0), 128)   
        wireframe = mesh.material.properties.get("wireframe", 0)   
        twosided = mesh.material.properties.get("twosided", 1)   

        setattr(mesh.material, "gl_mat", glGenLists(1))   
        glNewList(mesh.material.gl_mat, GL_COMPILE)

        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive)
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
        glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)

        glEndList()




    def renders(self,shadow):
        if self.whatShape == 0:
            self.meshRender(shadow)
        elif self.whatShape == 1:
            print("drawBox")
            #self.boxRender(self.boxSize)
        else:
            input("eerror mesh type")
    
    def meshRender(self,shadow):

        for mesh in self.scene.meshes:
            self.drawMesh(mesh,shadow)

    def drawMesh(self, mesh, shadow):
        #glPushMatrix()
        #OpenGL row major
        #print(mesh.faces)
        #input()
        self.apply_material(mesh.material,shadow)

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


    def apply_material(self, mat,shadow):
        if shadow:
             #if not hasattr(mat, "gl_mat"):
            diffuse = numpy.array(mat.properties.get("diffuse", [0.8, 0.8, 0.8, 0]))
            specular = numpy.array(mat.properties.get("specular", [0., 0., 0., 0]))
            ambient = numpy.array(mat.properties.get("ambient", [0.2, 0.2, 0.2, 0]))
            emissive = numpy.array(mat.properties.get("emissive", [0., 0., 0., 0]))   
            shininess = min(mat.properties.get("shininess", 1.0), 128)   
            wireframe = mat.properties.get("wireframe", 1)   
            twosided = mat.properties.get("twosided", 1)   

            setattr(mat, "gl_mat", glGenLists(1))   
            glNewList(mat.gl_mat,GL_COMPILE)

            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
            glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)

            glEndList()

    
        else:
            diffuse = numpy.array(mat.properties.get("diffuse", [192/255, 192/255, 192/255, 1.0]))
            specular = numpy.array(mat.properties.get("specular", [0/255, 0/255, 0/255, 1.0]))
            ambient = numpy.array(mat.properties.get("ambient", [192/255, 192/255, 192/255, 1.0]))
            emissive = numpy.array(mat.properties.get("emissive", [0., 0., 0., 1.0]))   
            shininess = min(mat.properties.get("shininess", 10000.0), 128)   
            wireframe = mat.properties.get("wireframe", 0)   
            twosided = mat.properties.get("twosided", 1)   
    
            setattr(mat, "gl_mat", glGenLists(1))   
            glNewList(mat.gl_mat,GL_COMPILE)
            glShadeModel(GL_SMOOTH)

            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
            glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)
    
            glEndList()
    
        glCallList(mat.gl_mat)

    def boxRender(self,_size):
        glPolygonMode(GL_FILL)
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
        input()
        while(i>=0):
            glBegin(GL_QUADS)
            glNormal3fv(_n[i])
            glVertex3fv(v[faces[i][0]])
            glVertex3fv(v[faces[i][1]])
            glVertex3fv(v[faces[i][2]])
            glVertex3fv(v[faces[i][3]])
            glEnd()
            i-=1
 
