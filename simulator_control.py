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

import settings as sets
import utility as utils

# Controller Initialization
ctrlfnc = utils.controller()

def main():
    ''' 描画インスタンス生成 '''
    for n in range(sets.Num_episodes):
        print('Loop Time: ', n+1)

        # Set initial parameters
        # (x, y, V, delta, theta, YR, beta, length, width, dt)
        utils.Car0.Initialization(0, 0, 18, 0, 0, 0, 0, 4.8, 1.7, sets.dt)
        utils.Car1.Initialization(30, 3.5, 16, 0, 0, 0, 0, 4.75, 1.75, sets.dt)
        utils.Car2.Initialization(-30, 3.5, 18, 0, 0, 0, 0, 4.75, 1.75, sets.dt)
        utils.Car3.Initialization(15, 0, 19, 0, 0, 0, 0, 4.75, 1.75, sets.dt)
        utils.Car4.Initialization(-15, 0, 18, 0, 0, 0, 0, 4.75, 1.75, sets.dt)

        '''　各ループ 制御演算 '''
        for m in range(1, sets.k_num):
            # Calculate control inputs using previous state values
            V = 18
            # delta = np.random.randn()
            delta = np.sin(np.pi*(m-1)*sets.Ts/2.5)*2e-2
            delta = ctrlfnc.PFC(m)

            # Calculate simulation values
            # print(m, ' time pasted.','\r', end='')
            utils.Car0.state_update(m, V, delta)
            utils.Car1.state_update(m, V, 0)
            utils.Car2.state_update(m, V, 0)
            utils.Car3.state_update(m, V, 0)
            utils.Car4.state_update(m, V, 0)

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

            for n in range(sets.k_num):
                # データをリストに保持
                csvlist = []
                csvlist.append(utils.Car0.t[n,0])
                csvlist.append(utils.Car0.X[n,0])
                csvlist.append(utils.Car0.Y[n,0])
                csvlist.append(utils.Car0.Vx[n,0])
                csvlist.append(utils.Car0.Vy[n,0])
                csvlist.append(utils.Car0.theta[n,0])
                csvlist.append(utils.Car0.YR[n,0])
                csvlist.append(utils.Car0.beta[n,0])
                csvlist.append(utils.Car0.delta[n,0])
                csvlist.append(utils.Car0.e2[n,0])
                csvlist.append(utils.Car0.e3[n,0])
                csvlist.append(utils.RefPath.X[n,0])
                csvlist.append(utils.RefPath.Y[n,0])
                csvlist.append(utils.RefPath.theta[n,0])
                # 出力
                writer.writerow(csvlist)

        if sets.MOV_FLAG:
            # Create new directory
            os.makedirs('AnimResults', exist_ok=True)
            ani = animation.FuncAnimation(sets.fig, utils.update, interval=sets.Ts*1e3, frames=sets.k_num)
            # plt.show()
            # ani.save(fname_anim, writer='imagemagick')
            ani.save('simulation.gif', writer="imagemagick")
            # ani.save('simulation.mp4', writer="imagemagick")
            utils.plot_result()
            print("Finished to create .csv and .gif files.")
        else:
            utils.plot_result()
            print("Finished to save .csv file.")


if __name__ == "__main__":
    main()

