import numpy as np
import scipy as sp
import scipy.sparse as sparse
import osqp

class MPCSolver:

    def __init__(self,Ad,Bd,Q,QN,R,N,x0,umin,umax,xmin,xmax,xr):

        self.Ad = Ad
        self.Bd = Bd
        [self.nx, self.nu] = Bd.shape

        self.Q  = Q
        self.QN = QN
        self.R  = R
        self.N = N
        self.x0 = x0
        self.umin = umin
        self.umax = umax
        self.xmin = xmin
        self.xmax = xmax
        self.xr = xr
        self.prob = osqp.OSQP()
        self.updateController()


    def updateDynamics(self,Ad,Bd):
        self.Ad = Ad
        self.Bd = Bd
        [self.nx, self.nu] = Bd.shape
        self.updateController()

    def updateMaxMin(self,umin,umax,xmin,xmax):
        self.umin = umin
        self.umax = umax
        self.xmin = xmax
        self.xmax = xmax
        self.updateController()

    def updateRef(self,xr):
        self.xr = xr
        self.updateController()

    def updateHorizon(self,N):
        self.N = N
        self.updateController()

    def updatex0(self,x0):
        self.x0 = x0
        self.updateController()

    def updateObjective(self,Q,QN,R):
        self.Q  = Q
        self.QN = QN
        self.R  = R
        self.updateController()

    def updateController(self):
        P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN,
                               sparse.kron(sparse.eye(self.N), self.R)])
        # - linear objective
        q = np.hstack([np.kron(np.ones(self.N), -self.Q.dot(self.xr)), -self.QN.dot(self.xr),
                       np.zeros(self.N * self.nu)])

        Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N + 1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)
        lineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmin), np.kron(np.ones(self.N), self.umin)])
        uineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmax), np.kron(np.ones(self.N), self.umax)])
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq])
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])
        self.prob.setup(P, q, A, l, u, warm_start=True, verbose = False)

    def solveMpc(self):
        res = self.prob.solve()
        ctrl = res.x[-self.N * self.nu:-(self.N - 1) * self.nu]

        return ctrl



    def update_bounds(self, umin, umax, xmin, xmax):
        """Updates the lower and upper bound for the inputs and states. """
        leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        ueq = leq

        lineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmin),
            np.kron(np.ones(self.N), self.umin)])
        uineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmax),
            np.kron(np.ones(self.N), self.umax)])

        l_new = np.hstack([leq, lineq])
        u_new = np.hstack([ueq, uineq])

        self.prob.update(l = l_new, u = u_new)


    def update_q(self, xr):
        """Updates the reference value, which results in updating q. """
        self.xr = xr

        q = np.hstack(
        [np.kron(np.ones(self.N), -self.Q.dot(self.xr)),
            -self.QN.dot(self.xr),
            np.zeros(self.N * self.nu)])

        self.prob.update(q = q)
