# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from control import ss, tf, interconnect, summing_junction
from control.matlab import lsim, step
from control import step_info
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
Kp = -1
Ki = -0.3
Kd = -0.1
N = 100
if N == 0:
	num = [Kd, Kp, Ki]
	den = [1, 0]
else:
	num = [Kp+Kd*N, Ki+Kp*N, Ki*N]
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

data = np.concatenate((tout.reshape(-1,1),yout),axis=1)

np.savetxt('true_pid_data.dat',data)

# Parsing results
Elevator_rad 	= (u0[0] + yout[:,10])
Throtlle 		= u0[1] * np.ones_like(tout)
XI_m 			= (y0[0] + yout[:,0])
ZI_m 			= (y0[1] + yout[:,1])
Theta_rad 		= (y0[2] + yout[:,2])
u_mps 			= (y0[3] + yout[:,3])
w_mps 			= (y0[4] + yout[:,4])
q_radps 		= (y0[5] + yout[:,5])
Alpha_deg 		= (y0[6] + yout[:,6]) * 180/np.pi
Gamma_deg 		= (y0[7] + yout[:,7]) * 180/np.pi
KTAS 			= (y0[8] + yout[:,8]) * 1.94384
Density_kgpm3 	= (y0[9] + yout[:,9])

Elevator_deg 	= (u0[0] + yout[:,10]) * 180/np.pi
Alt_ft 			= (y0[1] + yout[:,1]) * (-3.28084)
Theta_deg 		= (y0[2] + yout[:,2]) * 180/np.pi
q_degps 		= (y0[5] + yout[:,5]) * 180/np.pi

time_resp_info = step_info(sysdata=Theta_rad, T=t)

print(time_resp_info)

fig, axs = plt.subplots(nrows=3, ncols=3)
axs[0,0].plot(tout, Elevator_deg)
axs[0,0].set_ylabel('Elevator - [deg]')
axs[0,0].set_xlabel('Time - [s]')
axs[0,0].grid()
axs[0,1].plot(tout, Throtlle)
axs[0,1].set_ylabel('Throttle')
axs[0,1].set_xlabel('Time - [s]')
axs[0,1].grid()
axs[0,2].plot(tout, Theta_deg)
axs[0,2].set_ylabel('Pitch angle - [deg]')
axs[0,2].set_xlabel('Time - [s]')
axs[0,2].grid()
axs[1,0].plot(tout, KTAS)
axs[1,0].set_ylabel('True airspeed - [knot]')
axs[1,0].set_xlabel('Time - [s]')
axs[1,0].grid()
axs[1,1].plot(tout, Alt_ft)
axs[1,1].set_ylabel('Altitude - [ft]')
axs[1,1].set_xlabel('Time - [s]')
axs[1,1].grid()
axs[1,2].plot(tout, Density_kgpm3)
axs[1,2].set_ylabel('Air density - [kgpm3]')
axs[1,2].set_xlabel('Time - [s]')
axs[1,2].grid()
axs[2,0].plot(tout, Gamma_deg)
axs[2,0].set_ylabel('Path angle - [deg]')
axs[2,0].set_xlabel('Time - [s]')
axs[2,0].grid()
axs[2,1].plot(tout, Alpha_deg)
axs[2,1].set_ylabel('AoA - [deg]')
axs[2,1].set_xlabel('Time - [s]')
axs[2,1].grid()
axs[2,2].plot(tout, q_degps)
axs[2,2].set_ylabel('Pitch rate - [degps]')
axs[2,2].set_xlabel('Time - [s]')
axs[2,2].grid()
plt.suptitle('PID-controller pitch simulation')
plt.tight_layout()
plt.show()






