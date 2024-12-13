# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from control import ss, tf, interconnect, summing_junction
from control.matlab import lsim, step
import numpy as np
import matplotlib.pyplot as plt

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

ol_sys = ss(A, B, C, D,
			input=['elev'],
			outputs=['x','z','theta','u','w','q','alpha','gamma','v_TAS','rho'],
			states=['x','z','theta','u','w','q'],
			name='P')

x0 = np.array([0, -1524, 0, 62.3866, 0, 0])
u0 = np.array([-0.0032115, 0.6792])
y0 = np.array([0, -1524, 0, 623866, 0, 0, 0, 0, 62.3866, 1.0557])

# PID Controller
Kp = [-1,-1]
Ki = [-1,-0.3]
Kd = [0,-0.1]
N = 100
yout_list = []
tout_list = []
xout_list = []

for i in range(2):
	if N == 0:
		num = [Kd[i], Kp[i], Ki[i]]
		den = [1, 0]
	else:
		num = [Kp[i]+Kd[i]*N, Ki[i]+Kp[i]*N, Ki[i]*N]
		den = [1, N, 0]
	C_pid = tf(num, den, input=['e'], output=['elev'], name='C_pid')

	# Closed-loop with PID
	sumblk = summing_junction(inputs=['r','-theta'],output=['e'])
	cl_sys = interconnect([sumblk,C_pid,ol_sys],
						  input=['r'],
						  outputs=['x','z','theta','u','w','q','alpha','gamma','v_TAS','rho','elev'])

	# Simulation
	t = np.linspace(0,6,10000)
	ref_theta = 0.2 * np.ones_like(t) # 0.2 rad
	yout, tout, xout = lsim(cl_sys, ref_theta, t)

	# Appending data
	yout_list.append(yout)
	tout_list.append(tout)
	xout_list.append(xout)

# Parsing results for PID_1
Elevator_rad 	= (u0[0] + yout_list[0][:,10])
Throtlle 		= u0[1] * np.ones_like(tout_list[0])
XI_m 			= (y0[0] + yout_list[0][:,0])
ZI_m 			= (y0[1] + yout_list[0][:,1])
Theta_rad 		= (y0[2] + yout_list[0][:,2])
u_mps 			= (y0[3] + yout_list[0][:,3])
w_mps 			= (y0[4] + yout_list[0][:,4])
q_radps 		= (y0[5] + yout_list[0][:,5])
Alpha_deg 		= (y0[6] + yout_list[0][:,6]) * 180/np.pi
Gamma_deg 		= (y0[7] + yout_list[0][:,7]) * 180/np.pi
KTAS 			= (y0[8] + yout_list[0][:,8]) * 1.94384
Density_kgpm3 	= (y0[9] + yout_list[0][:,9])

Elevator_deg 	= (u0[0] + yout_list[0][:,10]) * 180/np.pi
Alt_ft 			= (y0[1] + yout_list[0][:,1]) * (-3.28084)
Theta_deg 		= (y0[2] + yout_list[0][:,2]) * 180/np.pi
q_degps 		= (y0[5] + yout_list[0][:,5]) * 180/np.pi

# Parsing results for PID_5
Elevator_rad2 	= (u0[0] + yout_list[1][:,10])
Throtlle2 		= u0[1] * np.ones_like(tout_list[1])
XI_m2 			= (y0[0] + yout_list[1][:,0])
ZI_m2 			= (y0[1] + yout_list[1][:,1])
Theta_rad2 		= (y0[2] + yout_list[1][:,2])
u_mps2 			= (y0[3] + yout_list[1][:,3])
w_mps2 			= (y0[4] + yout_list[1][:,4])
q_radps2 		= (y0[5] + yout_list[1][:,5])
Alpha_deg2 		= (y0[6] + yout_list[1][:,6]) * 180/np.pi
Gamma_deg2 		= (y0[7] + yout_list[1][:,7]) * 180/np.pi
KTAS2 			= (y0[8] + yout_list[1][:,8]) * 1.94384
Density_kgpm32 	= (y0[9] + yout_list[1][:,9])

Elevator_deg2 	= (u0[0] + yout_list[1][:,10]) * 180/np.pi
Alt_ft2 		= (y0[1] + yout_list[1][:,1]) * (-3.28084)
Theta_deg2 		= (y0[2] + yout_list[1][:,2]) * 180/np.pi
q_degps2 		= (y0[5] + yout_list[1][:,5]) * 180/np.pi


fig, axs = plt.subplots(nrows=2, ncols=1)
axs[0].plot(tout_list[0], Elevator_deg, tout_list[1], Elevator_deg2)
axs[0].legend(['PID_1','PID_5'])
axs[0].set_ylabel('Elevator - [deg]')
axs[0].set_xlabel('Time - [s]')
axs[0].grid()
axs[1].plot(tout_list[0], Theta_deg, tout_list[1], Theta_deg2)
axs[1].legend(['PID_1','PID_5'])
axs[1].set_ylabel('Pitch angle - [deg]')
axs[1].set_xlabel('Time - [s]')
axs[1].grid()
plt.suptitle('System response: PID_1 vs PID_5')
plt.tight_layout()
plt.show()






