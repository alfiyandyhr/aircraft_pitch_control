# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from pitch_gym_env import LinearPIDAircraftPitchEnv
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
import numpy as np
import torch

dt = 0.01
t_end = 6
N = 100 # time filter coefficient
target_pitch = 0.2 # rad
n_neurons_list = [64, 128, 256]
batch_size_list = [64, 128, 256]
save_model = False

env = LinearPIDAircraftPitchEnv(dt=dt, t_end=t_end, N=N)

for n_neurons in n_neurons_list:
	for batch_size in batch_size_list:
		save_folder = f'trained_models/neurons_{n_neurons}/batchsize_{batch_size}'
		onedrive_folder = '/Users/alfiyandyhr/OneDrive - HKUST Connect/Control_Project/trained_models'
		agent = PPO.load(f"{save_folder}/best_model.zip")
		policy_net = agent.policy.mlp_extractor.policy_net
		action_net = agent.policy.action_net
		actor_net = torch.nn.Sequential(
				policy_net,
				action_net
			)
		if save_model:
			torch.save(actor_net, f"{save_folder}/pytorch_model.pt")
			torch.save(actor_net, f"{onedrive_folder}/pytorch_model_n{n_neurons}_b{batch_size}.pt")

# Sanity check
x0 = np.array([0, 0, 0, 0, 0, 0, target_pitch])
observation, info = env.reset(x0=x0)
X0 = torch.from_numpy(observation).float()
action = agent.predict(observation, deterministic=True)[0]
print(action)
with torch.no_grad():
	out = np.clip(actor_net(torch.from_numpy(observation).float()).numpy(),-1.0,1.0)
print(out)
if action.all() == out.all():
	print('action == out')
	print('Sanity check complete!')