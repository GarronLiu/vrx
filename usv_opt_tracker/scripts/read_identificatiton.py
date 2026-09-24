import rosbag
import pandas as pd
from std_msgs.msg import Float64MultiArray
from datetime import datetime

bag_path = '/home/garronliu/2_Tracking_control/identification_2025-08-24-17-06-33.bag'
topic_name = '/tracker/acceleration_error'

data_list = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # msg.data 是一个长度为8的数组
        udot, vdot, rdot, u, v, r, t_left, t_right = msg.data[:8]
        data_list.append([
            t.to_sec(), udot, vdot, rdot, u, v, r, t_left, t_right
        ])

# 获取当前时间
current_time = datetime.now().strftime("# %a %b %d %H:%M:%S %Y")

# 定义文件头部信息
header = f"""{current_time}

# input dimensionality
5

# covariance function
CovSum(CovSEard, CovNoise)

# log-hyperparameter
1.560628737 1.940313351 0.1225281073 0.6544047328 0.6544047328 0.7783035935 -2.981304048  

# data (target value in first column) u v r tl tr"""

with open('udot_train.txt', 'w') as f:
    f.write(header + '\n')
    for item in data_list:
        # 只保存udot和后面的输入数据 u, v, r, t_left, t_right
        udot, u, v, r, t_left, t_right = item[1], item[4], item[5], item[6], item[7], item[8]
        f.write(f"{udot} {u} {v} {r} {t_left} {t_right}\n")

with open('vdot_train.txt', 'w') as f:
    f.write(header + '\n')
    for item in data_list:
        vdot, u, v, r, t_left, t_right = item[2], item[4], item[5], item[6], item[7], item[8]
        f.write(f"{vdot} {u} {v} {r} {t_left} {t_right}\n")

with open('rdot_train.txt', 'w') as f:
    f.write(header + '\n')
    for item in data_list:
        rdot, u, v, r, t_left, t_right = item[3], item[4], item[5], item[6], item[7], item[8]
        f.write(f"{rdot} {u} {v} {r} {t_left} {t_right}\n")

import matplotlib.pyplot as plt

# 提取时间和各个变量
time = [item[0] for item in data_list]
udot_vals = [item[1] for item in data_list]
vdot_vals = [item[2] for item in data_list]
rdot_vals = [item[3] for item in data_list]

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time, udot_vals, label='udot')
plt.ylabel('udot')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, vdot_vals, label='vdot', color='orange')
plt.ylabel('vdot')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, rdot_vals, label='rdot', color='green')
plt.xlabel('Time (s)')
plt.ylabel('rdot')
plt.legend()

plt.tight_layout()
plt.show()

