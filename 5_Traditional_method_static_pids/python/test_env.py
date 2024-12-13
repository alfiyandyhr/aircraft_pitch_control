# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from pitch_gym_env import LinearPIDAircraftPitchEnv
import matplotlib.pyplot as plt
import numpy as np

# Baseline PID with time filter
Kp = -1
Ki = -0.3
Kd = -0.1
N = 100

dt = 0.01
t_end = 6
target_pitch = 0.2 #rad

env = LinearPIDAircraftPitchEnv(dt=dt, t_end=t_end, N=N)

x0 = np.array([0, 0, 0, 0, 0, 0, target_pitch])
observation, info = env.reset(x0=x0)
# observation, info = env.reset(seed=42)
# print(observation, info)

# Initialization
x0 = np.array([0, -1524, 0, 62.3866, 0, 0])
u0 = np.array([-0.0032115, 0.6792])
y0 = np.array([0, -1524, 0, 62.3866, 0, 0, 0, 0, 62.3866, 1.0557])
e0 = target_pitch - x0[2]
t = np.zeros(int(t_end/dt)) # including t_start and t_end
x = np.zeros((int(t_end/dt),7))
y = np.zeros((int(t_end/dt),11))
e = np.zeros_like(t)
gains = np.zeros((int(t_end/dt),3))
episode_over = False
i = 0
while not episode_over:
	# Calculating action
	action = np.array([Kp, Ki, Kd])
	action = 2/3 * (action + 3.0) - 1.0 # rescaling

	# Journaling
	t[i] = info['t']
	x[i] = info['x']
	y[i] = info['y']
	e[i] = info['e']
	gains[i] = action
	i += 1

	observation, reward, terminated, truncated, info = env.step(action, verbose=False)
	episode_over = terminated or truncated

# Not including t_start and t_end
t = t[1:-1]
x = x[1:-1]
y = y[1:-1]
e = e[1:-1]
gains = gains[1:-1]

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

# True pid data
true_pid_data = np.genfromtxt('true_pid_data.dat')
t2 = true_pid_data[:,0]
y2 = true_pid_data[:,1:]

Elevator_rad2 	= (u0[0] + y2[:,10])
Throtlle2 		= u0[1] * np.ones_like(t2)
XI_m2 			= (y0[0] + y2[:,0])
ZI_m2 			= (y0[1] + y2[:,1])
Theta_rad2 		= (y0[2] + y2[:,2])
u_mps2 			= (y0[3] + y2[:,3])
w_mps2 			= (y0[4] + y2[:,4])
q_radps2 		= (y0[5] + y2[:,5])
Alpha_deg2 		= (y0[6] + y2[:,6]) * 180/np.pi
Gamma_deg2 		= (y0[7] + y2[:,7]) * 180/np.pi
KTAS2 			= (y0[8] + y2[:,8]) * 1.94384
Density_kgpm32 	= (y0[9] + y2[:,9])

Elevator_deg2 	= Elevator_rad2 * 180/np.pi
Alt_ft2 		= ZI_m2 * (-3.28084)
Theta_deg2 		= Theta_rad2 * 180/np.pi
q_degps2 		= q_radps2 * 180/np.pi

fig, axs = plt.subplots(nrows=3, ncols=3)
axs[0,0].plot(t2, Elevator_deg2, t, Elevator_deg)
axs[0,0].set_ylabel('Elevator - [deg]')
axs[0,0].set_xlabel('Time - [s]')
axs[0,0].grid()
axs[0,1].plot(t, Throtlle, t2, Throtlle2)
axs[0,1].set_ylabel('Throttle')
axs[0,1].set_xlabel('Time - [s]')
axs[0,1].grid()
axs[0,2].plot(t, Theta_deg, t2, Theta_deg2)
axs[0,2].set_ylabel('Pitch angle - [deg]')
axs[0,2].set_xlabel('Time - [s]')
axs[0,2].grid()
axs[1,0].plot(t, KTAS, t2, KTAS2)
axs[1,0].set_ylabel('True airspeed - [knot]')
axs[1,0].set_xlabel('Time - [s]')
axs[1,0].grid()
axs[1,1].plot(t, Alt_ft, t2, Alt_ft2)
axs[1,1].set_ylabel('Altitude - [ft]')
axs[1,1].set_xlabel('Time - [s]')
axs[1,1].grid()
axs[1,2].plot(t, Density_kgpm3, t2, Density_kgpm32)
axs[1,2].set_ylabel('Air density - [kgpm3]')
axs[1,2].set_xlabel('Time - [s]')
axs[1,2].grid()
axs[2,0].plot(t, Gamma_deg, t2, Gamma_deg2)
axs[2,0].set_ylabel('Path angle - [deg]')
axs[2,0].set_xlabel('Time - [s]')
axs[2,0].grid()
axs[2,1].plot(t, Alpha_deg, t2, Alpha_deg2)
axs[2,1].set_ylabel('AoA - [deg]')
axs[2,1].set_xlabel('Time - [s]')
axs[2,1].grid()
axs[2,2].plot(t, q_degps, t2, q_degps2)
axs[2,2].set_ylabel('Pitch rate - [degps]')
axs[2,2].set_xlabel('Time - [s]')
axs[2,2].grid()
plt.suptitle('PID-controller pitch simulation')
plt.tight_layout()
plt.show()
