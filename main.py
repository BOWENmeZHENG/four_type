from automate import run_model
import os, shutil
import numpy as np
from datetime import date


num = 1
seed = 45
material = 'AL_F'  # 'AL' or 'AL_A' or 'AL_F'
folder_name = f'{date.today()}_num_{num}_mat_{material}_seed_{seed}'
os.makedirs(folder_name, exist_ok=True)

np.random.seed(seed)
r_out = 0.1 * np.random.rand(num) + 0.2  # range: [0.2, 0.3]
r_in = r_out - (0.05 * np.random.rand(num) + 0.05)  # range: r_out - [0.05, 0.1]
width = 0.05 + 0.1 * np.random.rand(num)  # # range: [0.05, 0.15]
spoke_width = 0.02 * np.random.rand(num) + 0.02  # range: [0.02, 0.04]
num_spokes = np.random.randint(low=2, high=7, size=num)
load = 1000 + (100000 - 1000) * np.random.rand(num)  # range: [1e3, 1e5]
init_angle = 90 * np.random.rand(num)  # range: [0, 90]


for i in range(num):
    run_model(r_out=r_out[i], r_in=r_in[i], width=width[i],
              spoke_width=spoke_width[i], num_spokes=num_spokes[i], init_angle=init_angle[i],
              material_name=material, load=load[i], meshsize=0.02, vis=True)

# move csv files to folder
files = os.listdir('./')
for file in files:
    if file.endswith('.csv'):
        shutil.move(file, f'{folder_name}/{file}')
