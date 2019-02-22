import numpy as np
import copy
import math

class Matrix:
    def identityMat4x4():
        data = np.zeros((4,4))
        for i in range(0,4):
            data[i][i]=1

        return data

    def linear(data):
        ldata = copy.deepcopy(data[0:3,0:3])

        return ldata

    def col(data,i):
        cData = copy.deepcopy(data[0:,i])

        return cData

    def UnitY():
        data = np.zeros((1,3))
        data[0][1] = 1

        return data

    def size(data):
        sum = 0
        for i in range(0,3):
            #print(i)
            sum = sum + data[i]*data[i]
            #print(sum)
        #print(sum)
        #quit()
        sum = math.sqrt(sum)

        return sum
    def normalize(data):
        result = []
        sum = Matrix.size(data)
        if np.abs(sum) < 1e-6:
            print(data)
            input()
            sum = 1
        for i in data:
            result.append(i/sum)

        return result
    def size_2D(data):
        sum = 0
        for i in data:
            for j in i:
                sum = sum + j*j
        #print(sum)
        #quit()
        sum = math.sqrt(sum)
        return sum
        
    def normalize_2D(data):
        result = []
        sum = Matrix.size_2D(data)
        for i in data:
            for j in i:
                result.append(j/sum)
                #print("555")

        #print(result)

        return result
    def setTranslation(data, trans):
        data[0:3,3] = trans

    def setlinearCol(data, i, setData):
        #print(setData)
        #quit()
        data[0:3,i] = setData

    def getTranslation(data):
        return copy.deepcopy(data[0:3,3])

    def multTrans(transM, pos):
        pos = np.append(pos,1)
        pos =np.dot(transM, pos)
        return pos[0:3]

    def getNormal(a,b,c):
        ab = np.subtract(b,a)
        ac = np.subtract(c,a)
        nor = np.cross(ab,ac)
        nnor = Matrix.normalize(nor)

        return nnor

if __name__ == "__main__":
    data = Matrix.identityMat4x4()
    data2 =Matrix.linear(data)
    print(data)
    print(data2)
    data2[1][0] = 3
    
    data3 = Matrix.col(data2,0)
    data3[0]=5
    print(data)
    print(data2)
    print(data3)    

    Unityy = Matrix.UnitY()

    print(Unityy)
    
    data4=Matrix.normalize(data3)
    print(data3)
    print(data4)

    newData = Matrix.identityMat4x4()

    trans = [3,5,7]
    xx = [1,2,3]
    yy = [4,5,6]
    zz = [7,8,9]

    xx = Matrix.normalize(xx)

    Matrix.setTranslation(newData,trans)

    Matrix.setlinearCol(newData,0,xx)
    Matrix.setlinearCol(newData,1,yy)
    Matrix.setlinearCol(newData,2,zz)

    print(newData)

    trrr = Matrix.getTranslation(newData)

    print(trrr)

