# Written by Alfiyandy Hariansyah, HKUST -- December 2024
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

n_neurons_list = [64, 128, 256]
batchsize_list = [64, 128, 256]

eval_mean_reward_list = []
train_loss_list = []
train_value_loss_list = []

for n_neurons in n_neurons_list:
	for batchsize in batchsize_list:
		data1 = pd.read_csv(f'trained_models/neurons_{n_neurons}/batchsize_{batchsize}/eval_mean_reward.csv')
		data2 = pd.read_csv(f'trained_models/neurons_{n_neurons}/batchsize_{batchsize}/train_loss.csv')
		data3 = pd.read_csv(f'trained_models/neurons_{n_neurons}/batchsize_{batchsize}/train_value_loss.csv')
		eval_mean_reward_list.append([data1.Step.to_numpy(), data1.Value.to_numpy()])
		train_loss_list.append([data2.Step.to_numpy(), data2.Value.to_numpy()])
		train_value_loss_list.append([data3.Step.to_numpy(), data3.Value.to_numpy()])

fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(8,6), sharex=True)
for i in range(9):
	axs[0].plot(eval_mean_reward_list[i][0], eval_mean_reward_list[i][1])
axs[0].grid()
axs[0].set_ylabel('Eval_mean_rew')
for i in range(9):
	axs[1].plot(train_loss_list[i][0], train_loss_list[i][1])
axs[1].grid()
axs[1].set_ylabel('Action_net_loss')
for i in range(9):
	axs[2].plot(train_value_loss_list[i][0], train_value_loss_list[i][1])
axs[2].grid()
axs[2].set_xlabel('Timestep')
axs[2].set_ylabel('Value_net_loss')
fig.suptitle('Training and validation progress',fontsize=15)
fig.legend(['N64_B64','N64_B128','N64_B256','N128_B64','N128_B128','N128_B256','N256_B64','N256_B128','N256_B256'],
			ncols=3, loc='upper center', bbox_to_anchor=(0.5,0.94))
plt.subplots_adjust(left=0.1, bottom=0.1, right=0.97, top=0.8, hspace=0.1)
# plt.tight_layout()
plt.show()
# plt.savefig('figures/training_and_validation_progress.pdf',format='pdf')

print('--- Required training timesteps ---')
for i in range(3):
	for j in range(3):
		k = i*3+j
		print(f'N{n_neurons_list[i]}_B{batchsize_list[j]}: {eval_mean_reward_list[k][0][-1]}')