from control.matlab import ss, lsim
import numpy as np
import matplotlib.pyplot as plt

A = np.array([[0, 0, 62.39, 1, 0, 0],
			  [0, 0, 0, 0, 1, 0],
			  [0, 0, 0, 0, 0, 1],
			  [0, 9.807, 0, -0.1582, -0.103, -61.8],
			  [0, 0, 0, -0.3765, -11.57, 2.272],
			  [0, 0, 0, 0.137, -0.3595, -1.159]])
B = np.array([[0, 0],
			  [0, 0],
			  [0, 0],
			  [0, -5.953],
			  [-50.19, 3.178],
			  [-7.202, -8.754]])
C = np.array([[1, 0, 0, 0, 0, 0],
			  [0, 1, 0, 0, 0, 0],
			  [0, 0, 1, 0, 0, 0],
			  [0, 0, 0, 1, 0, 0],
			  [0, 0, 0, 0, 1, 0],
			  [0, 0, 0, 0, 0, 1],
			  [0, 0, 0, 0.016, 0, 0],
			  [0, 0, 1, 0, 0.016, 0]])
D = np.zeros((8,2))

sys = ss(A, B, C, D)
x0 = np.array([0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0])
y0 = np.array([0, 0, 0, 0, 0, 0, 0, 0])

t = np.linspace(0,60,10000)
D_u = np.zeros((t.shape[0],2))
D_delta_r = np.zeros_like(t)
for i in range(len(D_delta_r)):
	if t[i] >= 10 and t[i]<=12:
		D_delta_r[i] = 1.0 * np.pi/180
	if t[i] > 12 and t[i]<=14:
		D_delta_r[i] = -1.0 * np.pi/180
D_delta_a = np.zeros_like(t)
D_u[:,0] = D_delta_a
D_u[:,1] = D_delta_r

# Simulation
yout, tout, xout = lsim(sys, D_u, t)

# Parsing results
Aileron_deg 	= (u0[0] + D_delta_a) * 180/np.pi
Rudder_deg 		= (u0[1] + D_delta_r) * 180/np.pi
YI_m 			= (y0[0] + yout[:,0])
Phi_deg 		= (y0[1] + yout[:,1]) * 180/np.pi
Psi_deg 		= (y0[2] + yout[:,2]) * 180/np.pi
v_mps 			= (y0[3] + yout[:,3])
p_degps 		= (y0[4] + yout[:,4]) * 180/np.pi
r_degps 		= (y0[5] + yout[:,5]) * 180/np.pi
Beta_deg 		= (y0[6] + yout[:,6]) * 180/np.pi
Zeta_deg 		= (y0[7] + yout[:,7]) * 180/np.pi

fig, axs = plt.subplots(nrows=3, ncols=3)
axs[0,0].plot(tout, Rudder_deg)
axs[0,0].set_ylabel('Rudder - [deg]')
axs[0,0].set_xlabel('Time - [s]')
axs[0,0].grid()
axs[0,1].plot(tout, Aileron_deg)
axs[0,1].set_ylabel('Aileron - [deg]')
axs[0,1].set_xlabel('Time - [s]')
axs[0,1].grid()
axs[0,2].plot(tout, Psi_deg)
axs[0,2].set_ylabel('Yaw angle - [deg]')
axs[0,2].set_xlabel('Time - [s]')
axs[0,2].grid()
axs[1,0].plot(tout, Phi_deg)
axs[1,0].set_ylabel('Roll angle - [deg]')
axs[1,0].set_xlabel('Time - [s]')
axs[1,0].grid()
axs[1,1].plot(tout, p_degps)
axs[1,1].set_ylabel('Roll rate - [degps]')
axs[1,1].set_xlabel('Time - [s]')
axs[1,1].grid()
axs[1,2].plot(tout, r_degps)
axs[1,2].set_ylabel('Yaw rate - [degps]')
axs[1,2].set_xlabel('Time - [s]')
axs[1,2].grid()
axs[2,0].plot(tout, Zeta_deg)
axs[2,0].set_ylabel('Track angle - [deg]')
axs[2,0].set_xlabel('Time - [s]')
axs[2,0].grid()
axs[2,1].plot(tout, Beta_deg)
axs[2,1].set_ylabel('Sideslip angle - [deg]')
axs[2,1].set_xlabel('Time - [s]')
axs[2,1].grid()
axs[2,2].plot(tout, YI_m)
axs[2,2].set_ylabel('Inertial position Y - [m]')
axs[2,2].set_xlabel('Time - [s]')
axs[2,2].grid()
plt.suptitle('Lateral dynamics under rudder doublet')
plt.tight_layout()
plt.show()






