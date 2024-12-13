# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from pitch_gym_env import LinearPIDAircraftPitchEnv
from control import step_info
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
import matplotlib.pyplot as plt
import gymnasium as gym
import numpy as np
import torch

dt = 0.01
t_end = 6
N = 100 # time filter coefficient
target_pitch = 0.2 # rad
do_training = False # set this to True for training mode
do_test = True # set this to True for testing mode
use_ppo_agent = True # if not, the controller will be PID with static gain
n_neurons = 64
batch_size = 64
plot_trajectory = True
plot_reward_trajectory = True
plot_gains_trajectory = True
Kp, Ki, Kd = -1.0, -0.3, -0.1 # if not use_ppo_agent, these gains will be used

env = LinearPIDAircraftPitchEnv(dt=dt, t_end=t_end, N=N)

# Expensive!! Train only once!
if do_training:
	eval_env = LinearPIDAircraftPitchEnv(dt=dt, t_end=t_end, N=N)
	save_folder = f'trained_models/neurons_{n_neurons}/batchsize_{batch_size}'
	policy_kwargs = dict(net_arch=dict(pi=[n_neurons, n_neurons], vf=[n_neurons, n_neurons]))
	# Stop training when the model reaches the reward threshold
	callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=580, verbose=1)
	eval_callback = EvalCallback(eval_env, eval_freq=600,
								 best_model_save_path=f'./{save_folder}',
								 log_path=f'./{save_folder}',
								 callback_on_new_best=callback_on_best,
								 deterministic=True, render=False, verbose=1)
	agent = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, batch_size=batch_size, verbose=1, tensorboard_log=f'./{save_folder}')
	agent.learn(total_timesteps=int(1e6), log_interval=1, callback=eval_callback)
	# agent.save(f'{save_folder}/adaptive_pid.model')

if do_test:
	x0 = np.array([0, 0, 0, 0, 0, 0, target_pitch])
	observation, info = env.reset(x0=x0)
	# observation, info = env.reset(seed=42)
	# observation, info = env.reset()
	# print(observation, info)

	if use_ppo_agent:
		# agent = PPO.load(f'trained_models/neurons_{n_neurons}/batchsize_{batch_size}/adaptive_pid.model')
		agent = PPO.load(f'trained_models/neurons_{n_neurons}/batchsize_{batch_size}/best_model.zip')

	# Initialization
	x0 = np.array([0, -1524, 0, 62.3866, 0, 0])
	u0 = np.array([-0.0032115, 0.6792])
	y0 = np.array([0, -1524, 0, 62.3866, 0, 0, 0, 0, 62.3866, 1.0557])

	episode_over = False
	i = 0
	t = np.zeros(int(t_end/dt)) # including t_start and t_end
	x = np.zeros((int(t_end/dt),7))
	y = np.zeros((int(t_end/dt),11))
	e = np.zeros_like(t)
	gains = np.zeros((int(t_end/dt),3))
	reward_trajectory = np.zeros(int(t_end/dt))

	while not episode_over:
		# Calculating action
		if use_ppo_agent:
			action = agent.predict(observation, deterministic=True)[0]
		else:
			action = np.array([Kp, Ki, Kd]) # Kp, Ki, Kd, N=100
			action = 2/3 * (action + 3.0) - 1.0 # rescaling

		# Journaling
		t[i] = info['t']
		x[i] = info['x']
		y[i] = info['y']
		e[i] = info['e']
		gains[i] = 1.5 * (action + 1.0) - 3.0
		i += 1

		observation, reward, terminated, truncated, info = env.step(action)
		episode_over = terminated or truncated
		reward_trajectory[i-1] = reward

	# Journaling
	t[-1] = info['t']
	x[-1] = info['x']
	y[-1] = info['y']
	e[-1] = info['e']
	gains[-1] = 1.5 * (action + 1.0) - 3.0
	i += 1

	# Not including t_start
	t = t[1:]
	x = x[1:]
	y = y[1:]
	e = e[1:]
	gains = gains[1:]

	# Printing total reward obtained
	print(f'Total reward = {np.sum(reward_trajectory)}')

	# Parsing results
	Elevator_rad 	= (u0[0] + y[:,10])
	Throtlle 		= u0[1] * np.ones_like(t)
	XI_m 			= (y0[0] + y[:,0])
	ZI_m 			= (y0[1] + y[:,1])
	Theta_rad 		= (y0[2] + y[:,2])
	u_mps 			= (y0[3] + y[:,3])
	w_mps 			= (y0[4] + y[:,4])
	q_radps 		= (y0[5] + y[:,5])
	Alpha_deg 		= (y0[6] + y[:,6]) * 180/np.pi
	Gamma_deg 		= (y0[7] + y[:,7]) * 180/np.pi
	KTAS 			= (y0[8] + y[:,8]) * 1.94384
	Density_kgpm3 	= (y0[9] + y[:,9])

	Elevator_deg 	= Elevator_rad * 180/np.pi
	Alt_ft 			= ZI_m * (-3.28084)
	Theta_deg 		= Theta_rad * 180/np.pi
	q_degps 		= q_radps * 180/np.pi

	time_resp_info = step_info(sysdata=Theta_rad, T=t)
	print(time_resp_info)

if plot_trajectory:
	fig, axs = plt.subplots(nrows=3, ncols=3)
	axs[0,0].plot(t, Elevator_deg)
	axs[0,0].set_ylabel('Elevator - [deg]')
	axs[0,0].set_xlabel('Time - [s]')
	axs[0,0].grid()
	axs[0,1].plot(t, Throtlle)
	axs[0,1].set_ylabel('Throttle')
	axs[0,1].set_xlabel('Time - [s]')
	axs[0,1].grid()
	axs[0,2].plot(t, Theta_deg)
	axs[0,2].set_ylabel('Pitch angle - [deg]')
	axs[0,2].set_xlabel('Time - [s]')
	axs[0,2].grid()
	axs[1,0].plot(t, KTAS)
	axs[1,0].set_ylabel('True airspeed - [knot]')
	axs[1,0].set_xlabel('Time - [s]')
	axs[1,0].grid()
	axs[1,1].plot(t, Alt_ft)
	axs[1,1].set_ylabel('Altitude - [ft]')
	axs[1,1].set_xlabel('Time - [s]')
	axs[1,1].grid()
	axs[1,2].plot(t, Density_kgpm3)
	axs[1,2].set_ylabel('Air density - [kgpm3]')
	axs[1,2].set_xlabel('Time - [s]')
	axs[1,2].grid()
	axs[2,0].plot(t, Gamma_deg)
	axs[2,0].set_ylabel('Path angle - [deg]')
	axs[2,0].set_xlabel('Time - [s]')
	axs[2,0].grid()
	axs[2,1].plot(t, Alpha_deg)
	axs[2,1].set_ylabel('AoA - [deg]')
	axs[2,1].set_xlabel('Time - [s]')
	axs[2,1].grid()
	axs[2,2].plot(t, q_degps)
	axs[2,2].set_ylabel('Pitch rate - [degps]')
	axs[2,2].set_xlabel('Time - [s]')
	axs[2,2].grid()
	plt.suptitle('PID-controller pitch simulation')
	plt.tight_layout()
	plt.show()

if plot_reward_trajectory:
	plt.plot(t, Theta_rad, label='Pitch angle - [rad]')
	plt.plot(t, Elevator_rad, label='Elevator deflection - [rad]')
	plt.plot(t, reward_trajectory[:-1], label='Reward')
	plt.legend(loc='center right')
	plt.grid()
	plt.show()

if plot_gains_trajectory:
	fig, axs = plt.subplots(nrows=3, ncols=1)
	axs[0].plot(t, np.round(gains[:,0],3))
	axs[0].set_xlabel('Time - [s]')
	axs[0].set_ylabel('Kp')
	axs[0].grid()
	axs[1].plot(t, np.round(gains[:,1],3))
	axs[1].set_xlabel('Time - [s]')
	axs[1].set_ylabel('Ki')
	axs[1].grid()
	axs[2].plot(t, np.round(gains[:,2],3))
	axs[2].set_xlabel('Time - [s]')
	axs[2].set_ylabel('Kd')
	axs[2].grid()
	plt.suptitle('PID gains trajectory')
	plt.show()