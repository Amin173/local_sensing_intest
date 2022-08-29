from faulthandler import dump_traceback_later
import rospy
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
import numpy as np
import pandas as pd
from time import time, sleep



# Where to save
saving_path = "slam_.csv"

#TIming 
t0 = time()
t1 = t0

def id2str(id):
    if id < 10:
        return '0' + str(id)
    else:
        return str(id)

offset_counter = 0
offset_initialized = False
imus_initialized = [False] * 12
data_tmp = {}
for i in range(12):
    data_tmp[id2str(i)+'x'] = 0.
    data_tmp[id2str(i)+'y'] = 0.
    data_tmp[id2str(i)+'a'] = 0.
    data_tmp[id2str(i)+'at'] = 0.

p = pd.DataFrame(columns=list(data_tmp.keys()))

offsets = np.zeros(12)

def callback(msg, idx):
    global p
    global t1
    global data_tmp
    global t0
    global imus_initialized

    quat_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    # Convert quaternion to euler angle
    euler = -euler_from_quaternion(quat_list)[-1]

    if offset_initialized:
        #euler -= offsets[int(idx)]
        pass

    data_tmp[idx+'a'] = euler
    imus_initialized[int(idx)] = True

    t0 = time()

def callback2(msg):
    global offset_initialized
    global data_tmp
    global offset_counter
        
    data = eval(msg.data)

    if not offset_initialized and sum(imus_initialized) == 12:
        offset_counter += 1
        if offset_counter > 100.:
            offset_initialized = True
        else:
            for i in range(12):
                imus_initialized[i] = False
                idx = id2str(i)
                offsets[i] += (data_tmp[idx+'a'] - data[idx][-1] * np.pi / 180) / 100.
                
    else:
        for i in range(12):
            idx = id2str(i)
            data_tmp[idx+'x'] = data[idx][0]
            data_tmp[idx+'y'] = data[idx][1]
            data_tmp[idx+'at'] = data[idx][2] * np.pi / 180



rospy.init_node('Test', anonymous=False)
for i in range(12):
    rospy.Subscriber("bot%s/imu/data" % id2str(i), Imu, callback, id2str(i))

rospy.Subscriber("state", String, callback2)


while np.abs(t1 - t0) < 10.:
    t1 = time()
    sleep(.1)
    print(data_tmp)
    if offset_initialized:
        p = p.append(data_tmp, ignore_index=True)


p.to_csv(saving_path)