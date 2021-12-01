from enum import Enum
import numpy as np

class StateType(Enum):
    UNKNOWN = 0
    XY      = 1
    XYZ     = 2

class MotionModel(Enum):
    UNKNOWN = 0
    CV      = 1
    CA      = 2
    CT      = 3

def h(x, y):
    mat = np.zeros([2, 1])
    pass
    return mat


# matrice représentant la cinétique du mouvement
def F(periode=0.0, dim=4, motionModelType=MotionModel.UNKNOWN):
    mat = np.identity(dim)

    if dim == 4 and motionModelType == MotionModel.CV:
       pass

    return mat

def Q(T=0.0, dim=4, motionModelType=MotionModel.UNKNOWN, noise=0.0):
    mat = np.identity(dim)
    COV = []
    if dim == 4 and motionModelType == MotionModel.CV:
       pass
     

    return  COV