# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt

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


