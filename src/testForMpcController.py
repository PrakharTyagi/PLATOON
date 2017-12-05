from mpc_controller_frame import Mpc_controller
import numpy as np
import numpy as np

import osqp
import scipy as sp
import scipy.sparse as sparse

mpc = Mpc_controller()




nsim = 5

x = 1
y = 0
r = 1
yaw = np.pi/2
list = np.zeros(5)

#def solveMpcTest(self,x,ref,Ad,Bd,QQ,QQN,RR,N,umin,umax,xmin,xmax):
Ad = sparse.csc_matrix([
    [0.5]
])
Bd = sparse.csc_matrix([
    [1]
])

xref = 10
QQ = sparse.diags([0.1])
QQN = QQ
RR = 0.1 * sparse.eye(1)
N = 10
ref = 5;
umin = np.array([-1.])
umax = np.array([1.])
xmin = np.array([-5])
xmax = np.array([10])

for i in range(nsim):
    ctrl2 = mpc.solveMpcTest(x,ref,Ad,Bd,QQ,QQN,RR,N,umin,umax,xmin,xmax)
    print('hej')
    x = Ad*x+Bd*ctrl2
    print(x)
#for i in range(nsim):
    #ctrl1 = mpc.solveMpc(x,y,r,yaw)
   # print(ctrl1)
    #z = mpc.getNextState(ctrl1)

    #list[i] = ctrl1





