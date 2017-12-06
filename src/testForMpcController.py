from mpc_controller_frame import Mpc_controller
import numpy as np
import numpy as np

import osqp
import scipy as sp
import scipy.sparse as sparse



x = 0
y = 0
r = 1
yaw = 0

#def solveMpcTest(self,x,ref,Ad,Bd,QQ,QQN,RR,N,umin,umax,xmin,xmax):
Ad = sparse.csc_matrix([
    [1., 0., 0.],
    [0., 1., 0.],
    [0., 0., 1]
])
Bd = sparse.csc_matrix([
    [0.5, .0],
    [0.5, 0.],
    [0., 0.5]
])


Q = sparse.diags([0.1, 0.1, 0.1])
QN = Q
R = 0.1 * sparse.eye(2)
umin = np.array([0,0])
umax = np.array([1,1])
xmin = np.array([-10,-10,-10])
xmax = np.array([10,10,10])
x0 = np.zeros(3)

N = 3



xref = np.array([6,6,6])
mpc = Mpc_controller(Ad,Bd,Q,QN,R,N,x0,umin,umax,xmin,xmax,xref)




#umin = np.array([-1.])
#umax = np.array([1.])
#xmin = np.array([-5])
#xmax = np.array([10])

nsim = 10

#print mpc.Q
##print mpc.QN
#print (mpc.Ad)
for i in range(nsim):
    ctrl = mpc.solveMpc()
    x0 = Ad.dot(x0)+Bd.dot(ctrl)
   #x=1*x+mpc.h*ctrl(0)
    mpc.updatex0(x0)
    print(ctrl)
    print(x0)

#for i in range(nsim):
    #ctrl1 = mpc.solveMpc(x,y,r,yaw)
   # print(ctrl1)
    #z = mpc.getNextState(ctrl1)

    #list[i] = ctrl1





