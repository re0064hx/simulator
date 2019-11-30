''' ball_animation.py '''
import os
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

'''
データの作成
'''
def create_ball_data(time_count=128, time_interval=0.1, box_size=10):
    data = np.zeros((time_count, 2*2))
    position = np.zeros(2)
    velocity = np.zeros(2)

    #位置と速度の初期設定
    for i in range(2):
        position[i] = random.uniform(0, box_size)
        velocity[i] = random.uniform(0.5, 0.8)

    #データの作成
    for t in range(time_count):
        pos_next = position + velocity * time_interval

        for i in range(2):
            if pos_next[i] < 0 or pos_next[i] > box_size:
                velocity[i] = velocity[i] * -1.0

        position = position + velocity * time_interval

        data[t] = np.copy(np.hstack((position,velocity)))

    return data

'''
アニメーションの表示
'''
def show_animation(data, box_size, fname=None):
    figure = plt.figure()
    area = figure.add_subplot(111, autoscale_on=False)
    area.xaxis.set_ticks(range(box_size + 1))
    area.yaxis.set_ticks(range(box_size + 1))
    area.grid()

    balls = []
    for i in range(len(data)):
        draw, = area.plot([], [], 'o', color='blue', markersize=10)
        balls.append(draw)

    def update(t):
        for i in range(len(data)):
            p = data[t][:2]
            balls[i].set_data(p[0], p[1])
        return balls

    ani = animation.FuncAnimation(figure, update, interval=1, frames=len(data), blit=False )

    if fname:
        try:
            path_file_name = '{0}.gif'.format(os.path.join(os.path.dirname(__file__), fname ))
            ani.save(path_file_name, writer='imagemagick', fps=30)
            print('Save Finished!')
        except Exception as ex:
            print('save error')
            pass

    plt.show()

'''
メイン
'''
if __name__ == '__main__':
    time_count = 150
    box_size = 10

    data = create_ball_data(time_count=time_count, box_size=box_size)
    show_animation(data=data, box_size=box_size, fname='test')
