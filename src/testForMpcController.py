from mpc_controller_frame import Mpc_controller
import numpy as np


mpc = Mpc_controller()




nsim = 100

x = 1
y = 0
r = 1
yaw = np.pi/2
for i in range(nsim):
    ctrl1 = mpc.solveMpc(x,y,r,yaw)




