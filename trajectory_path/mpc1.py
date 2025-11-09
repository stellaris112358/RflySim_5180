import numpy as np
from numpy.linalg import matrix_power as mp
import cvxpy as cp
import time

class uav_mpc:
    def __init__(self):
        self.T = 1 / 30
        self.umin=np.array([-0.2, -0.6, -0.1])
        self.umax=np.array([0.5, 0.6, 0.1])
        self.q = np.array([1, 1, 1, 1, 1])
        self.r = np.array([0.4, .4, 0.4])

    def block_diag_np(self, *arrays):
        shapes = np.array([a.shape for a in arrays])
        out_shape = shapes.sum(0)
        out = np.zeros(out_shape, dtype=arrays[0].dtype)
        r, c = 0, 0
        for a in arrays:
            nr, nc = a.shape
            out[r:r+nr, c:c+nc] = a
            r += nr
            c += nc
        return out

    def step(self, Ur, Xr, X, Np):
        q=np.diag(self.q)
        Q=self.block_diag_np(q,q,q,q,q)
        R=np.diag(self.r)
        R=self.block_diag_np(R, R, R, R, R)

        A=np.array([[1, 0, 0, -Ur[0, 0]*self.T*Xr[3]/(Xr[4] + 1e-8), Ur[0, 0]*self.T],
            [0, 1, 0, Ur[0, 0]*self.T, -Ur[0, 0]*self.T*Xr[4]/(Xr[3] + 1e-8)],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1-Ur[0, 1]*Xr[3]/(Xr[4] + 1e-8)*self.T, Ur[0, 1]*self.T],
            [0, 0, 0, -Ur[0, 1]*self.T, Ur[0, 1]*Xr[4]/(Xr[3] + 1e-8)*self.T]])
        B=np.array([[self.T*Xr[4], 0, 0],
                    [self.T*Xr[3], 0, 0],
                    [0, 0, self.T],
                    [0, self.T*Xr[4], 0],
                    [0, -self.T*Xr[3], 0]])
        M=np.vstack([mp(A, 1), mp(A, 2), mp(A, 3), mp(A, 4), mp(A, 5)])
        i, j = A.shape[0], B.shape[1]
        z = np.zeros((i, j))
        C = np.block([
            [mp(A, 0) @ B, z, z, z, z],
            [mp(A, 1) @ B, mp(A, 0) @ B, z, z, z],
            [mp(A, 2) @ B, mp(A, 1) @ B, mp(A, 0) @ B, z, z],
            [mp(A, 3) @ B, mp(A, 2) @ B, mp(A, 1) @ B, mp(A, 0) @ B, z],
            [mp(A, 4) @ B, mp(A, 3) @ B, mp(A, 2) @ B, mp(A, 1) @ B, mp(A, 0) @ B]
            ])

        H=(C.T@Q@C+R)/2
        X_ = np.array(X - Xr).reshape(1, -1)
        f=2*X_@M.T@Q@C
        lb=[]
        ub=[]
        for i in range(Np):
            lb.append(self.umin - Ur[i, :])
            ub.append(self.umax - Ur[i, :])
        lb = np.concatenate(lb)
        ub = np.concatenate(ub)

        x = cp.Variable(3*Np)
        objective = cp.Minimize(0.5 * cp.quad_form(x, H) + f[0] @ x)
        constraints = [
            x >= lb,
            x <= ub
            ]
        prob = cp.Problem(objective, constraints)
        prob.solve()
        U_=np.array(x.value[0:3])
        return Ur[0, :] + U_
