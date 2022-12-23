import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv('2022-12-23_num_1_mat_AL_F_seed_45/ro_0.299_ri_0.221_w_0.078_sw_0.022_n_5_m_AL_F_l_91549.4_rot_43.3_nodes.csv')
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
sca = ax.scatter(data.x, data.y, data.z, c=data.nodetype)
plt.colorbar(sca, pad=0.1)
plt.show()