import math
from klampt.math import so3

## Get rotation matrix from zyx Euler angles
def euler_zyx_mat(theta):
    """For the zyx euler angles theta=(rz,ry,rx), produces a matrix A such that
    A*dtheta is the angular velocities when dtheta is the rate of change of the
    euler angles"""
    eu = [0,0,1]
    ev = [0,1,0]
    ew = [1,0,0]
    Ru = so3.rotation([0,0,1],theta[0])
    Rv = so3.rotation([0,1,0],theta[1])
    col1 = eu
    col2 = so3.apply(Ru,ev)
    col3 = so3.apply(Ru,so3.apply(Rv,ew))
    #col1 = [0,0,1]
    #col2 = [c0 -s0 0] [0] = [-s0]
    #       [s0 c0  0]*[1]   [c0 ]
    #       [0  0   1] [0]   [0  ]
    #col3 = Ru*[c1  0 s1] [1] = Ru*[c1 ] = [c1c0]
    #          [0   1 0 ]*[0]      [0  ]   [c1s0]
    #          [-s1 0 c1] [0]      [-s1]   [-s1 ]
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]    
    #return [col1[2], col2[2], col3[2], col1[1], col2[1], col3[1], col1[0], col2[0], col3[0]]
    phi = theta[0]
    tht = theta[1]
    psi = theta[2]
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    ctht = math.cos(tht)
    stht = math.sin(tht)
    cpsi = math.cos(psi)
    spsi = math.sin(psi)
    rotMat = [cphi*ctht, sphi*cpsi+cphi*stht*spsi, sphi*spsi-cphi*stht*cpsi, -sphi*ctht, cphi*cpsi-sphi*stht*spsi, cphi*spsi+sphi*stht*cpsi, stht,-spsi*ctht, cpsi*ctht]
    return rotMat

## The function below is incomplete
def euler_zyx_mat_inv(theta):
    """Returns the inverse of the matrix returned by the above procedure"""
    c0 = math.cos(theta[0])
    s0 = math.sin(theta[0])
    c1 = math.cos(theta[1])
    s1 = math.sin(theta[1])
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]
    #det(A) = -c1
    #A^-1 = 1/c1*[ s1c0 s0s1 c1  ]
    #            [-c1s0 c1c0 0   ]
    #            [ c0   s0   0   ]
    #A^-1*A = 1/c1*[c1 -s0s1c0+c0s0s1 s1c1c0^2+s1c1s0^2-c1s1 ] = [1 0 0]
    #              [0   c1s0^2+c1c0^2 -c0c1^2s0+s0c1^2c0     ]   [0 1 0]
    #              [0   -s0c0+s0c0    c1c0^2+c1s0^2          ]   [0 0 1]
    sec1 = 1.0/c1
    return [[c0*s1/c1,s0*s1/c1,1],
            [-s0,c0,0],
            [c0/c1,s0/c1,0]]


