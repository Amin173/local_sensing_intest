import glob
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from bagpy import bagreader
from scipy.signal import savgol_filter
from numpy import array

# %%
# path = "/home/amin/catkin_ws/src/local_sensing_intest/bags/maze4/2021-11-24_15h15/"
# bag_file = "*42.bag"
path = "/home/amin/catkin_ws/src/local_sensing_intest/bags/maze4/2021-11-23_22h39/"
bag_file = "*15.bag"
# # path = "/home/amin/catkin_ws/src/local_sensing_intest/bags/maze4/2021-11-23_22h04/"
# bag_file = "*07.bag"
bag_path = glob.glob(path + bag_file)[0]
b = bagreader(bag_path)

# %%
b.topic_table

# %%
state = b.message_by_topic('state')
df_state = pd.read_csv(state)

des_dir = b.message_by_topic('locomotion_stats')
df_dir = pd.read_csv(des_dir)

control_cmd = b.message_by_topic('control_cmd')
df_cmd = pd.read_csv(control_cmd)

# %%
def key(i):
    if i < 10:
        key = str(0) + str(i)
    else:
        key = str(i)
    return key


# %%
t_interp = np.arange(0, 700, 0.1)
R = []
for i in range(12):
    dist = b.message_by_topic('bot' + key(i) + '/dist/data')
    df_dist = pd.read_csv(dist)
    r = np.interp(t_interp, np.array(df_dist["Time"] - df_dist["Time"][0]), np.array(df_dist["range"]))
    if np.size(R) == 0:
        R = r
    else:
        R = np.vstack((R, r))

# %%
xm = []
ym = []
Theta = []
t = np.array(df_state["Time"])
t = t - t[0]
for i in df_state.index:
    x = []
    y = []
    theta = []
    for j in range(12):
        x = np.append(x, eval(df_state["data"][i])[key(j)][0]-eval(df_state["data"][i])["12"][0])
        y = np.append(y, -(eval(df_state["data"][i])[key(j)][1]-eval(df_state["data"][i])["12"][1]))
        theta = np.append(theta, (eval(df_state["data"][i])[key(j)][2]-eval(df_state["data"][i])["12"][2]))
    theta = -(theta + np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90]))
    xm = np.append(xm, np.mean(x))
    ym = np.append(ym, np.mean(y))
    if np.size(Theta) == 0:
        Theta = theta
    else:
        Theta = np.vstack((Theta, theta))
Theta = Theta * np.pi / 180
des_dir_array = []
for i in df_dir.index:
    if np.size(des_dir_array) == 0:
        des_dir_array = np.array(eval(df_dir["data"][i])['dir'])
    else:
        des_dir_array = np.vstack((des_dir_array, np.array(eval(df_dir["data"][i])['dir'])))
cmd = []
for i in df_cmd.index:
    cmd_i = []
    for j in range(12):
        cmd_i = np.append(cmd_i, eval(df_cmd["data"][0])[key(j)][0])
    if np.size(cmd) == 0:
        cmd = cmd_i
    else:
        cmd = np.vstack((cmd, cmd_i))
vx = np.gradient(xm, t)
vy = np.gradient(ym, t)
ax = np.gradient(vx, t)
ay = np.gradient(vy, t)
v = np.sqrt(vx ** 2 + vy ** 2)

# %%
heading_dirs = np.hstack((np.cos(Theta[2:]), np.sin(Theta[2:])))
print("heading_dirs shape:", np.shape(heading_dirs))

# %%
rel_headings = np.zeros([np.shape(des_dir_array)[0], 12])
for i in range(12):
    rel_headings[:, i] = heading_dirs[:, i] * des_dir_array[:, 0] - heading_dirs[:, i + 12] * des_dir_array[:, 1]
print("rel_headings shape:", np.shape(rel_headings))
print("range_data shape:", np.shape(R))

# %%
rel_headings_interp = []
for i in range(12):
    interp = np.interp(t_interp, t[2:] - t[2], rel_headings[:, i])
    if np.size(rel_headings_interp) == 0:
        rel_headings_interp = interp
    else:
        rel_headings_interp = np.vstack((rel_headings_interp, interp))

# %%
normal_distances = R * rel_headings_interp
normal_distances = normal_distances * (normal_distances > 0) * (rel_headings_interp >= np.cos(np.pi / 12))
plt.plot(normal_distances.T, 'o', alpha=0.3)
norm_dist_max = np.max(normal_distances.T, axis=1)
for id1, i in enumerate(norm_dist_max):
    if (norm_dist_max[id1 - 1] - i) > 0.1:
        norm_dist_max[id1] = norm_dist_max[id1 - 1]

norm_dist_filtered = savgol_filter(norm_dist_max, 151, 3)
v_dist = np.gradient(norm_dist_filtered, t_interp)
plt.plot(norm_dist_max, 'k', alpha=0.5)
plt.plot(norm_dist_filtered, 'r')
plt.show()

# %%
t_odom = np.array(df_dir["Time"])
t_odom = t_odom - t_odom[0]
m = 0
v_odom = np.mean(abs(rel_headings * cmd), axis=1)
v_hat = savgol_filter(v, 51, 3)

x_odom = []
y_odom = []
V = 0.7
t0 = 0
for i in range(t_odom.shape[0] - 1):
    V = 0.7
    if t_odom[i] > (t0 + 6):
        x_odom = np.append(x_odom, xm[abs(t - t_odom[i]) == np.min(abs(t - t_odom[i]))] - xm[0])
        y_odom = np.append(y_odom, ym[abs(t - t_odom[i]) == np.min(abs(t - t_odom[i]))] - ym[0])
        t0 = t_odom[i]
    else:
        if x_odom == []:
            x_odom = np.append(x_odom, V * (t_odom[i + 1] - t_odom[0]) * des_dir_array[i, 0])
            y_odom = np.append(y_odom, V * (t_odom[i + 1] - t_odom[0]) * des_dir_array[i, 1])
        else:
            x_odom = np.append(x_odom, x_odom[-1] + V * (t_odom[i + 1] - t_odom[i]) * des_dir_array[i, 0])
            y_odom = np.append(y_odom, y_odom[-1] + V * (t_odom[i + 1] - t_odom[i]) * des_dir_array[i, 1])

plt.plot(xm - xm[0], ym - ym[0], ':')
plt.plot(x_odom, y_odom)
plt.show()

# %%
coeffs = np.zeros(3)
coeffs[0] = np.linalg.norm(v_odom) ** 2
coeffs[1] = -2 * np.linalg.norm(v_odom) * np.linalg.norm(v_hat)
coeffs[2] = np.linalg.norm(v_hat) ** 2
np.roots(coeffs)

# %%
plt.plot(t[30:], v_hat[30:], label="Ground Truth")
# plt.plot(t_interp, savgol_filter((v_dist - v_dist[0]) * 50, 651, 3))
plt.plot(t_odom, -3.8 * v_odom + 3.25, ":", label="Linear Model")
# plt.ylim([0, 2])
plt.legend()
plt.ylabel("Velocity [cm/s]")
plt.xlabel("Time [s]")
plt.show()

# %%
v_linear_model_int = np.cumsum(-3.8 * v_odom + 3.25)
v_ground_truth_int = np.cumsum(v_hat[30:])

plt.plot(t[30:], v_ground_truth_int, label="Ground Truth")
# plt.plot(t_interp, savgol_filter((v_dist - v_dist[0]) * 50, 651, 3))
plt.plot(t_odom, v_linear_model_int, ":", label="Linear Model")
# plt.ylim([0, 2])
plt.legend()
plt.ylabel("Velocity [cm/s]")
plt.xlabel("Time [s]")
plt.show()

# %%
plt.plot(Theta)
plt.show()
