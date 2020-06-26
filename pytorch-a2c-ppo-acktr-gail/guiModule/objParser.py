import numpy as np

class objParser:
    def __init__(self,):
        self.vertexArrays = []
        self.TriangleVertexArrays = []
        self.TriangleNormalArrays = []
        self.vertexNormals  = []
        self.NormalArrays = []

    def openObj(self,_objFileName):
        self.objFile = open(_objFileName, "r")

    def closeObj(self,):
        self.objFile.close()

    def parseObj(self,):
        for line in self.objFile:
            split = line.split()
            
            if len(split) == 0:
                continue
            
            if split[0] =='o':
                continue
            
            if split[0] == "v":
                self.vertexArrays.append(self.parseVertex(line))
            elif split[0] == "vt":
                continue
            elif split[0] == "vn":
                self.vertexNormals.append(self.parseVertexNormal(line))
            elif split[0] == "f":
                vertex, normal = self.parseTrinagleFace(line)
                self.TriangleVertexArrays.append(vertex)
                self.TriangleNormalArrays.append(normal)
        
        for i in range(len(self.vertexArrays)):
            self.NormalArrays.append([])

        for i in range(len(self.TriangleNormalArrays)):
            for j in range(3):
                print(self.TriangleNormalArrays[i][j])
                print("vv", self.TriangleVertexArrays[i][j])
                self.NormalArrays[self.TriangleVertexArrays[i][j]] = self.vertexNormals[self.TriangleNormalArrays[i][j]]
                print(self.NormalArrays)
                print(self.TriangleVertexArrays)
                #input()

                
    def parseVertex(self, vertexLine):
        splitVertex = vertexLine.split(" ")

        x = float(splitVertex[1])
        y = float(splitVertex[2])
        z = float(splitVertex[3])

        return [x,y,z]

    def parseVertexNormal(self, vertexNormalLine):
        splitVertexNormal = vertexNormalLine.split(" ")
        print(splitVertexNormal)
        x = float(splitVertexNormal[1])
        y = float(splitVertexNormal[2])
        z = float(splitVertexNormal[3])

        return [x,y,z]

    def parseTrinagleFace(self, triangleFaceLine):
        splitTriFace = triangleFaceLine.split(" ")
        vertexss = []
        normalss = []
        for splited in splitTriFace[1:]:
            v, vn = self.parseVertexNormalTexture(splited)
            vertexss.append(v)
            normalss.append(vn)
        return vertexss, normalss
        
    def parseVertexNormalTexture(self, line):
        split = line.split("/")
        return int(split[0])-1, int(split[2])-1

    def print_v(self,):
        print("rpitn_vStart")
        print(self.vertexArrays)
        print(self.TriangleVertexArrays)
        #print(self.TriangleNormalArrays)
        #print(self.vertexNormals)
        print("print_vEnd")


if __name__ == "__main__":
    parser = objParser()
    parser.openObj("./arrow.obj")
    parser.parseObj()
    parser.print_v()
    parser.closeObj()

