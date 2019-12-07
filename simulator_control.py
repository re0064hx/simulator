# -*- coding: utf-8 -*-
import os
import csv
import time
import math
import datetime
import shutil
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

Num_episodes = 1
MOV_FLAG = True
Tf = 30
Ts = 0.05
dt = Ts
k_num = round(Tf/Ts)
max_x = 5
min_x = -5
max_y = 30
min_y = -30
fig = plt.figure(figsize=(3,15))
ax = fig.add_subplot(111)

# 車両クラスの定義
class Vehicle():
    def __init__(self):
        # normal coordinate to vehicle coordinate (X to Y and Y to X)
        self.t = np.zeros([k_num, 1])
        self.X = np.zeros([k_num, 1])
        self.Y = np.zeros([k_num, 1])
        self.Vx = np.zeros([k_num, 1])
        self.Vy = np.zeros([k_num, 1])
        self.V = np.zeros([k_num, 1])
        self.theta = np.zeros([k_num, 1])
        self.YR = np.zeros([k_num, 1])
        self.beta = np.zeros([k_num, 1])
        self.delta = np.zeros([k_num, 1])
        self.e2 = np.zeros([k_num, 1])
        self.e3 = np.zeros([k_num, 1])

    def Initialization(self, init_x, init_y, init_V, init_delta, theta, YR, beta, length, width, dt):
        self.X[0, 0] = init_x
        self.Y[0, 0] = init_y
        self.V[0, 0] = init_V
        self.delta[0, 0] = init_delta
        self.theta[0, 0] = theta
        self.YR[0, 0] = YR
        self.beta[0, 0] = beta
        self.Vx[0, 0] = self.V[0, 0] * np.cos(self.beta[0, 0])
        self.Vy[0, 0] = self.V[0, 0] * np.sin(self.beta[0, 0])
        self.e2[0, 0] = Car0.Y[0, 0] - RefPath.Y[0, 0]
        self.e3[0, 0] = Car0.theta[0, 0] - RefPath.theta[0, 0]
        self.length = length
        self.width = width
        self.dt = dt

        self.Kf = 40000
        self.Kr = 70000
        self.I = 3500
        self.lf = 2.2
        self.lr = self.length - self.lf
        self.m = 2000

    def state_update(self, l, V, delta):
        #座標系に注意
        self.t[l, 0] = l*Ts
        self.delta[l, 0] = delta
        self.V[l, 0] = V
        self.Vx[l, 0] = self.V[l, 0] * np.cos(self.beta[l-1, 0])
        self.Vy[l, 0] = self.V[l, 0] * np.sin(self.beta[l-1, 0])
        self.single_track_model(l, delta)
        self.theta[l, 0] = self.theta[l-1, 0] + self.YR[l, 0] * self.dt
        self.X[l, 0] = self.X[l-1, 0] + self.V[l, 0]*np.cos(self.theta[l, 0] + self.beta[l, 0]) * self.dt # longitudinal coordinate value
        self.Y[l, 0] = self.Y[l-1, 0] + self.V[l, 0]*np.sin(self.theta[l, 0] + self.beta[l, 0]) * self.dt # lateral coordinate value
        self.e2[l, 0] = Car0.Y[l-1, 0] - RefPath.Y[l-1, 0]
        self.e3[l, 0] = Car0.theta[l-1, 0] - RefPath.theta[l-1, 0]

    def single_track_model(self, l, delta):
        beta_dot = 0
        YR_dot = 0

        beta_dot = -2*(self.Kf+self.Kr)/(self.m*self.V[l-1, 0]) * self.beta[l-1, 0] \
                - (self.m*self.V[l-1, 0] + 2*(self.lf*self.Kf-self.lr*self.Kr)/self.V[l-1, 0])/(self.m*self.V[l-1, 0]) * self.YR[l-1, 0] \
                + (2*self.Kf)/(self.m*self.V[l-1, 0]) * delta
        YR_dot = -2*(self.lf*self.Kf-self.lr*self.Kr)/self.I * self.beta[l-1, 0] \
                - 2*(np.power(self.lf,2)*self.Kf+np.power(self.lr,2)*self.Kr)/(self.I*self.V[l-1, 0]) * self.YR[l-1, 0] \
                + (2*self.lf*self.Kf)/self.I * delta
        self.beta[l, 0] = self.beta[l-1, 0] + beta_dot*self.dt
        self.YR[l, 0] = self.YR[l-1, 0] + YR_dot*self.dt

    # def step(self, V, YR):
        # self.state_update(V, YR)
        # state = np.array([self.x, self.y, self.vx, self.vy, self.theta])
        # state = np.reshape(state, [1, 5])

        # if abs(self.x) > 0.5:
        #     reward = -1
        #     done = True
        # elif abs(self.y) > 30:
        #     reward = 2
        #     done = True
        # else:
        #     reward = 1
        #     done = False

        # return state, reward, done

# PathPlannerクラスの定義
class PathPlanner():
    def __init__(self):
        self.X = np.zeros((k_num, 1))
        self.Y = np.zeros((k_num, 1))
        self.theta = np.zeros((k_num, 1))
        self.rho = np.zeros((k_num, 1))

    def path_generation(self):
        # csv_input = pd.read_csv('reference_path.csv')
        csv_input = pd.read_csv('oval_course.csv')
        Path = csv_input.values
        print(len(Path))
        for cnt in range(k_num):
            self.X[cnt, 0] = Path[cnt, 0]
            self.Y[cnt, 0] = Path[cnt, 1]
            self.theta[cnt, 0] = np.arctan2(self.Y[cnt, 0], self.X[cnt, 0])
            # self.rho =

# 車両制御コントローラの定義
class controller():
    def __init__(self):
        self.delta = 0

    def PFC(self, l):
        '''
        This function is calculate lateral control input
        based on Kanayama's method.
        '''
        K2 = 0.05
        K3 = 0.2
        rho = 0
        # e2 = RefPath.Y[l-1, 0] - Car0.Y[l-1, 0]
        # e3 = RefPath.theta[l-1, 0] - Car0.theta[l-1, 0]

        term1 = (Car0.Kf*Car0.lf - Car0.Kr*Car0.lr)/(Car0.Kf*Car0.V[l-1, 0]) * Car0.YR[l-1, 0]
        term2 = (Car0.Kf + Car0.Kr)/(Car0.Kf) * Car0.beta[l-1, 0]
        term3 = (Car0.m * Car0.V[l-1, 0])/(2*Car0.Kf) * (rho*(Car0.V[l-1, 0]*np.cos(Car0.e3[l-1, 0]))/(1-Car0.e2[l-1, 0]*rho) - K2*Car0.e2[l-1, 0]*Car0.V[l-1, 0] - K3*np.sin(Car0.e3[l-1, 0]))

        self.delta = term1 + term2 + term3
        return self.delta

def update(i):
    plt.cla()
    # Limitation of x-axis and y-axis
    ax.set_xlim(min_x + Car0.Y[i, 0], max_x + Car0.Y[i, 0])
    ax.set_ylim(min_y + Car0.X[i, 0], max_y + Car0.X[i, 0])
    # e3 = RefPath.theta[l-1, 0] - Car0.theta[l-1, 0]

    # 軸の縦横比, 正方形，単位あたりの長さを等しくする
    ax.set_aspect('equal')
    # self.change_aspect_ratio(ax, 1/5) # 横を1/5倍長く（縦を5倍長く）設定

    # 軸の名前設定
    ax.set_xlabel('Y [m]')
    ax.set_ylabel('X [m]')

    # その他グラフ仕様
    ax.invert_xaxis() # y軸（通常の座標におけるx軸）を反転
    ax.grid(True) # グリッド

    # rectangle
    rect_0 = patches.Rectangle((Car0.Y[i,0]-Car0.width/2, Car0.X[i,0]-Car0.length/2),Car0.width,Car0.length,angle=-Car0.theta[i, 0]*180/np.pi, ec='r', fill=False)
    rect_1 = patches.Rectangle((Car1.Y[i,0]-Car1.width/2, Car1.X[i,0]-Car1.length/2),Car1.width,Car1.length,angle=-Car1.theta[i, 0]*180/np.pi, ec='b', fill=False)
    rect_2 = patches.Rectangle((Car2.Y[i,0]-Car2.width/2, Car2.X[i,0]-Car2.length/2),Car2.width,Car2.length,angle=-Car2.theta[i, 0]*180/np.pi, ec='b', fill=False)
    rect_3 = patches.Rectangle((Car3.Y[i,0]-Car3.width/2, Car3.X[i,0]-Car3.length/2),Car3.width,Car3.length,angle=-Car3.theta[i, 0]*180/np.pi, ec='b', fill=False)
    rect_4 = patches.Rectangle((Car4.Y[i,0]-Car4.width/2, Car4.X[i,0]-Car4.length/2),Car4.width,Car4.length,angle=-Car4.theta[i, 0]*180/np.pi, ec='b', fill=False)
    ax.add_patch(rect_0)
    ax.add_patch(rect_1)
    ax.add_patch(rect_2)
    ax.add_patch(rect_3)
    ax.add_patch(rect_4)

def plot_result():
    # Create new directory
    now = datetime.datetime.now()
    os.makedirs('Figures/{0:%Y%m%d-%H%M%S}'.format(now), exist_ok=True)

    ## XY ##
    f0 = plt.figure()
    ax1 = f0.add_subplot(211)
    ax2 = f0.add_subplot(212)
    ax1.plot(Car0.t, Car0.X)
    ax2.plot(Car0.t, Car0.Y)
    # Axis settings
    ax1.set_xlabel('Time[s]')
    ax1.set_ylabel('X[m]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax1.grid(True)
    ax2.set_xlabel('Time[s]')
    ax2.set_ylabel('X[m]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax2.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/XY.png'.format(now))

    ## Vx Vy ##
    f1 = plt.figure()
    ax3 = f1.add_subplot(211)
    ax4 = f1.add_subplot(212)
    ax3.plot(Car0.t, Car0.Vx)
    ax4.plot(Car0.t, Car0.Vy)
    # Axis settings
    ax3.set_xlabel('Time[s]')
    ax3.set_ylabel('Vx[m/s]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax3.grid(True)
    ax4.set_xlabel('Time[s]')
    ax4.set_ylabel('Vy[m/s]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax4.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/VxVy.png'.format(now))

    ## YR Beta ##
    f2 = plt.figure()
    ax5 = f2.add_subplot(211)
    ax6 = f2.add_subplot(212)
    ax5.plot(Car0.t, Car0.YR)
    ax6.plot(Car0.t, Car0.beta)
    # Axis settings
    ax5.set_xlabel('Time[s]')
    ax5.set_ylabel('YR[rad/s]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax5.grid(True)
    ax6.set_xlabel('Time[s]')
    ax6.set_ylabel('Beta[rad]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax6.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/YR_Beta.png'.format(now))

    ## theta delta ##
    f3 = plt.figure()
    ax7 = f3.add_subplot(211)
    ax8 = f3.add_subplot(212)
    ax7.plot(Car0.t, Car0.theta)
    ax8.plot(Car0.t, Car0.delta)
    # Axis settings
    ax7.set_xlabel('Time[s]')
    ax7.set_ylabel('theta[rad]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax7.grid(True)
    ax8.set_xlabel('Time[s]')
    ax8.set_ylabel('delta[rad]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax8.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/theta_delta.png'.format(now))

    ## e2e3 ##
    f4 = plt.figure()
    ax9 = f4.add_subplot(211)
    ax10 = f4.add_subplot(212)
    ax9.plot(Car0.t, Car0.e2)
    ax10.plot(Car0.t, Car0.e3)
    # Axis settings
    ax9.set_xlabel('Time[s]')
    ax9.set_ylabel('e2[m]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax9.grid(True)
    ax10.set_xlabel('Time[s]')
    ax10.set_ylabel('e3[rad]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax10.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/e2e3.png'.format(now))

    ## e2e3 ##
    f5 = plt.figure()
    ax11 = f5.add_subplot(211)
    ax12 = f5.add_subplot(212)
    ax11.plot(RefPath.X, RefPath.Y)
    ax12.plot(Car0.X, Car0.Y)
    # Axis settings
    ax11.set_xlabel('Longitudinal displacement[m]')
    ax11.set_ylabel('Lateral displacement[m]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax11.grid(True)
    ax12.set_xlabel('Longitudinal displacement[m]')
    ax12.set_ylabel('Lateral displacement[m]')
    # ax1.set_xlim(-np.pi, np.pi)
    ax12.grid(True)
    # Save figure
    plt.savefig('Figures/{0:%Y%m%d-%H%M%S}/X-Y.png'.format(now))

    plt.close()


def main():
    ''' 描画インスタンス生成 '''
    for n in range(Num_episodes):
        print('Loop Time: ', n+1)

        # Set initial parameters
        # (x, y, V, delta, theta, YR, beta, length, width, dt)
        Car0.Initialization(0, 0, 18, 0, 0, 0, 0, 4.8, 1.7, dt)
        Car1.Initialization(30, 3.5, 16, 0, 0, 0, 0, 4.75, 1.75, dt)
        Car2.Initialization(-30, 3.5, 18, 0, 0, 0, 0, 4.75, 1.75, dt)
        Car3.Initialization(15, 0, 19, 0, 0, 0, 0, 4.75, 1.75, dt)
        Car4.Initialization(-15, 0, 18, 0, 0, 0, 0, 4.75, 1.75, dt)

        '''　各ループ 制御演算 '''
        for m in range(1, k_num):
            # Calculate control inputs using previous state values
            V = 18
            # delta = np.random.randn()
            delta = np.sin(np.pi*(m-1)*Ts/2.5)*2e-2
            delta = ctrlfnc.PFC(m)

            # Calculate simulation values
            # print(m, ' time pasted.','\r', end='')
            Car0.state_update(m, V, delta)
            Car1.state_update(m, V, 0)
            Car2.state_update(m, V, 0)
            Car3.state_update(m, V, 0)
            Car4.state_update(m, V, 0)

        print('Finish Calculation.')

        ''' Show simulation results '''
        # Create new directory
        os.makedirs('SimResults', exist_ok=True)
        now = datetime.datetime.now()
        # Create file name containing date imformation
        fname_result = "SimResults/{0:%Y%m%d-%H%M%S}.csv".format(now)
        fname_anim = "AnimResults/{0:%Y%m%d-%H%M%S}.gif".format(now)
        with open(fname_result, 'w') as f:
            writer = csv.writer(f, lineterminator='\n')

            for n in range(k_num):
                # データをリストに保持
                csvlist = []
                csvlist.append(Car0.t[n,0])
                csvlist.append(Car0.X[n,0])
                csvlist.append(Car0.Y[n,0])
                csvlist.append(Car0.Vx[n,0])
                csvlist.append(Car0.Vy[n,0])
                csvlist.append(Car0.theta[n,0])
                csvlist.append(Car0.YR[n,0])
                csvlist.append(Car0.beta[n,0])
                csvlist.append(Car0.delta[n,0])
                csvlist.append(Car0.e2[n,0])
                csvlist.append(Car0.e3[n,0])
                csvlist.append(RefPath.X[n,0])
                csvlist.append(RefPath.Y[n,0])
                csvlist.append(RefPath.theta[n,0])
                # 出力
                writer.writerow(csvlist)

        if MOV_FLAG:
            # Create new directory
            os.makedirs('AnimResults', exist_ok=True)
            ani = animation.FuncAnimation(fig, update, interval=Ts*1e3, frames=k_num)
            # plt.show()
            # ani.save(fname_anim, writer='imagemagick')
            ani.save('simulation.gif', writer="imagemagick")
            # ani.save('simulation.mp4', writer="imagemagick")
            plot_result()
            print("Finished to create .csv and .gif files.")
        else:
            plot_result()
            print("Finished to save .csv file.")



''' Create vehicle instance and reference path'''
Car0 = Vehicle()
Car1 = Vehicle()
Car2 = Vehicle()
Car3 = Vehicle()
Car4 = Vehicle()
RefPath = PathPlanner()
RefPath.path_generation()
ctrlfnc = controller()

if __name__ == "__main__":
    main()
