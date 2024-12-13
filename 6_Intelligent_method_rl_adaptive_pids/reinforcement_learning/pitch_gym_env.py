# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from os import path
from typing import Optional
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.registration import register

DEFAULT_X_HIGH = [0, 0, 0, 0, 0, 0, 0.5]

class LinearPIDAircraftPitchEnv(gym.Env):
	"""
	# Discrete PID control is implemented here

	# Observation space
		One observation variable: (e = target_pitch - observed_pitch)
			| Num | Observation 		 | Min  | Max |
			|-----|----------------------|------|-----|
			| 0   | e - normalized error | -inf | inf |

	# Action space/control input
		Three control variables: (PID gains)
			| Num | Action | Min | Max |
			|-----|--------|-----|-----|
			| 0   | Kp     | -3  | 0   |
			| 1   | Ki     | -3  | 0   |
			| 2   | Kd     | -3  | 0   |

	# State space
		Six state variables:
			| Num | States         			   | Min  | Max |
			|-----|----------------------------|------------|
			| 0   | x - position   			   | -inf | in  |
			| 1   | z - position   			   | -inf | inf |
			| 2   | theta - pitch  			   | -0.5 | 0.5 |
			| 3   | u - velocity   			   | 0    | inf |
			| 4   | w - velocity   			   | 0    | inf |
			| 5   | q - pitch rate 			   | -inf | inf |
			| 6   | target_theta - target pitch| -0.5 | 0.5 |

	# Rewards
		Reward = - normalized_error^2 - normalized_effort^2 - 10.0*int(terminated) + 1.0

	# Arguments
		dt 		: infinitesimal time - [s]
		t_start	: initial time - [s]
		t_end	: final time - [s]
		N 		: time filter coefficient
	"""
	def __init__(self, render_mode: Optional[str] = None, dt=0.01, t_start=0, t_end=10, N=100):

		# Clock
		self.dt = dt
		self.t_start = t_start
		self.t_end = t_end
		self.t  = t_start

		# Boundaries
		self.max_e =  np.inf
		self.min_e = -np.inf
		self.max_derivator = np.inf
		self.min_derivator = -np.inf
		self.max_integrator = np.inf
		self.min_integrator = -np.inf
		self.max_Kp, self.min_Kp = 1.0, -1.0
		self.max_Ki, self.min_Ki = 1.0, -1.0
		self.max_Kd, self.min_Kd = 1.0, -1.0

		self.max_u = 30/180*np.pi # max elev deflection

		# Default
		self.N = N # time filter coefficient

		# Integrator, prev_error, derivator PID
		self.integrator = None
		self.prev_error = None
		self.derivator = None
		self.prev_d_output = None

		self.render_mode = render_mode

		high_action = np.array([self.max_Kp, self.max_Ki, self.max_Kd], dtype=np.float32)
		low_action  = np.array([self.min_Kp, self.min_Ki, self.min_Kd], dtype=np.float32)

		high_observation = np.array([self.max_e], dtype=np.float32)
		low_observation  = np.array([self.min_e], dtype=np.float32)

		self.action_space = spaces.Box(low=low_action, high=high_action, shape=(3,), dtype=np.float32)
		self.observation_space = spaces.Box(low=low_observation, high=high_observation, shape=(1,), dtype=np.float32)

	def step(self, action, verbose=False):

		# Rescaling
		action = 1.5 * (action + 1.0) - 3.0

		# Unpacking action by agent
		Kp = action[0]
		Ki = action[1]
		Kd = action[2]

		e = self.curr_error 	# [e = target_pitch - observed_pitch]
		x = self.state 			# [x, z, theta, u, w, q, target_theta]

		# Open-loop linear dynamics
		A, B, C, D = self._linear_dynamics()

		# Calculate PID output
		p_output = Kp*e
		i_output = Ki*self.integrator*self.dt
		if Kd != 0:
			if self.N == 0 or None:
				d_output = Kd * (e - self.prev_error) / self.dt
			else:
				d_output = 1/(self.N*self.dt + 1)*self.prev_d_output + Kd*self.N/(self.N*self.dt + 1) * self.derivator
				self.prev_d_output = d_output
		else:
			d_output = 0.0
		pid_out = p_output + i_output + d_output
		pid_out = np.clip(pid_out, -self.max_u, self.max_u)
		if verbose:
			print(f'e:{e}, p:{p_output}, i:{i_output}, d:{d_output}, u:{pid_out}')

		# Update integrator and prev_error
		self.integrator += e
		self.prev_error = e

		# New state calculation
		new_x = x[:-1].reshape(-1,1) + (np.matmul(A,x[:-1].reshape(-1,1)) + B*pid_out)*self.dt
		new_x = np.concatenate((new_x,np.array([[x[-1]]])), axis=0)

		# New output
		new_y = np.matmul(C,new_x[:-1])
		new_y = np.append(new_y,pid_out)

		# Flattening the state and output
		self.state = new_x.reshape(-1)
		self.output = new_y.reshape(-1)

		# New error and update derivator
		self.curr_error = self.state[-1] - self.state[2]
		self.derivator = self.curr_error - self.prev_error

		if self.render_mode == 'human':
			pass

		# Exceeds bound if abs(pitch_angle) >= pi/2 = 90 deg
		if abs(self.state[2]) >= np.pi/2:
			terminated = True
		else:
			terminated = False

		# Truncated if t reaches t_end
		self.t += self.dt 
		if self.t >= self.t_end-self.dt:
			truncated = True
		else:
			truncated = False

		# Calculate reward
		reward = self._get_reward(pid_out, terminated)

		return self._get_obs(), reward, terminated, truncated, self._get_info()

	def reset(self, x0: Optional[list] = None, seed: Optional[int] = None):
		super().reset(seed=seed)

		# Setting the clock back to zero - IMPORTANT (easy to skip)
		self.t = self.t_start

		if x0 is None:
			high = np.array(DEFAULT_X_HIGH)
			low = -high # enforcing symmetric limits
			self.state = self.np_random.uniform(low=low, high=high)
		else:
			# Use user-defined initial states
			self.state = np.array(x0)

		# Calculate initial observation
		self.curr_error = self.state[-1] - self.state[2] # target_pitch - curr_pitch

		# Calculate initial output
		_, _, C, _, = self._linear_dynamics()
		self.output = np.matmul(C,self.state[:-1].reshape(-1,1))
		self.output = np.append(self.output,0) # assuming zero elev initially

		# Initial integrator, prev_error, derivator
		self.integrator = self.curr_error
		self.prev_error = 0.0
		self.derivator = self.curr_error - self.prev_error
		self.prev_d_output = 0.0

		if self.render_mode == 'human':
			pass

		return self._get_obs(), self._get_info()

	def _get_obs(self):
		percent_error = self.curr_error/self.state[-1]
		return np.array([percent_error])

	def _get_reward(self, effort, terminated):
		error_penalty = -self.curr_error**2 / self.state[-1]**2
		effort_penalty = -(effort/self.max_u)**2
		exceed_bound_penalty = -10*int(terminated)
		healthy_reward = 1.0
		# print(error_penalty, effort_penalty, exceed_bound_penalty, healthy_reward)
		reward = error_penalty + effort_penalty + exceed_bound_penalty + healthy_reward
		return reward

	def _get_info(self):
		return {'t':np.round(self.t,2), 'x':self.state, 'y':self.output, 'e':self.curr_error}

	def _linear_dynamics(self):
		# Linear model of aircraft pitch
		A = np.array([[0, 0, 0, 1, 0, 0],
					  [0, 0, -62.39, 0, 1, 0],
					  [0, 0, 0, 0, 0, 1],
					  [0, -0.0001, -9.807, -0.0477, 0.2388, 0],
					  [0, -0.0022, 0, -0.3152, -2.64, 60.9],
					  [0, 0, 0, 0.0005, -0.2494, -3.971]])
		B = np.array([[0],[0],[0],[1.91],[-13.69],[-33.99]])
		C = np.array([[1, 0, 0, 0, 0, 0],
					  [0, 1, 0, 0, 0, 0],
					  [0, 0, 1, 0, 0, 0],
					  [0, 0, 0, 1, 0, 0],
					  [0, 0, 0, 0, 1, 0],
					  [0, 0, 0, 0, 0, 1],
					  [0, 0, 0, 0, 0.016, 0],
					  [0, 0, 1, 0, -0.016, 0],
					  [0, 0, 0, 1, 0, 0],
					  [0, 0.0002, 0, 0, 0, 0]])
		D = np.zeros((10,1))
		return A, B, C, D

	def render(self):
		# to be implemented
		pass

	def close(self):
		if self.screen is not None:
			import pygame

			pygame.display.quit()
			pygame.quit()
			self.isopen = False

if __name__ == '__main__':
	register(
			id='gymnasium_env/LinearPIDAircraftPitch-v0',
			entry_point=LinearPIDAircraftPitchEnv
		)