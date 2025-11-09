import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import matplotlib.pyplot as plt


class UKF:
    def __init__(self, x0, dt=1/30):
        self.dt = dt

        # Sigma点生成器
        points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=0, sqrt_method=np.linalg.cholesky)

        # 创建UKF
        self.ukf = UKF(dim_x=4, dim_z=3, dt=dt, fx=self.fx, hx=self.hx, points=points)
        self.ukf.x = x0
        self.ukf.P = np.eye(4) * 0.1
        self.ukf.Q = np.eye(4) * 0.01
        self.ukf.R = np.eye(3) * 0.05


    # 运动模型
    def fx(self, x, dt, u):
        x_pos, y_pos, z_pos, yaw = x
        u_speed, vz, w = u

        x_next = x_pos + u_speed * np.cos(yaw) * dt
        y_next = y_pos + u_speed * np.sin(yaw) * dt
        z_next = z_pos + vz * dt
        yaw_next = yaw + w * dt

        return np.array([x_next, y_next, z_next, yaw_next])

    # 测量模型
    def hx(self, x):
        # 测量的是位置
        return x[:3]
    
    def run(self, x, u, z):
        x = np.array(x)
        u = np.array(u)
        z = np.array(z)

        self.ukf.predict(u=u)
        self.ukf.update(z)

        return self.ukf.x
    
if __name__ == "__main__":

    # 模拟数据
    N = 100
    true_states = []
    measurements = []
    estimates = []

    # # 真实初始状态
    # x_true = np.array([0.0, 0.0, 0.0, 0.0])

    # for i in range(N):
    #     # 控制输入
    #     u = np.array([1.0, 0.1, 0.05])  # u_speed, vz, w

    #     # 真实状态更新（加过程噪声）
    #     x_true = fx(x_true, dt, u) + np.random.multivariate_normal(np.zeros(4), Q)
    #     true_states.append(x_true.copy())

    #     # 生成测量（加测量噪声）
    #     z = hx(x_true) + np.random.multivariate_normal(np.zeros(3), R)
    #     measurements.append(z.copy())

    #     # UKF更新
    #     ukf.predict(u=u)
    #     ukf.update(z)
    #     estimates.append(ukf.x.copy())

    # # 转换为数组
    # true_states = np.array(true_states)
    # measurements = np.array(measurements)
    # estimates = np.array(estimates)

    # # 绘图
    # plt.figure(figsize=(10, 6))
    # plt.plot(true_states[:, 0], true_states[:, 1], label='True Path')
    # plt.scatter(measurements[:, 0], measurements[:, 1], c='r', s=10, label='Measurements')
    # plt.plot(estimates[:, 0], estimates[:, 1], '--', label='UKF Estimate')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('UKF Tracking with Control Input')
    # plt.legend()
    # plt.grid()
    # plt.show()