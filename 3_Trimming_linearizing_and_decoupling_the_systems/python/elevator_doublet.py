from control.matlab import ss, lsim
import numpy as np
import matplotlib.pyplot as plt

A = np.array([[0, 0, 0, 1, 0, 0],
			  [0, 0, -62.39, 0, 1, 0],
			  [0, 0, 0, 0, 0, 1],
			  [0, -0.0001, -9.807, -0.0477, 0.2388, 0],
			  [0, -0.0022, 0, -0.3152, -2.64, 60.9],
			  [0, 0, 0, 0.0005, -0.2494, -3.971]])
B = np.array([[0, 0],
			  [0, 0],
			  [0, 0],
			  [1.91, 1.462],
			  [-13.69, 0.0255],
			  [-33.99, -0.0146]])
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
D = np.zeros((10,2))

sys = ss(A, B, C, D)
x0 = np.array([0, -1524, 0, 62.3866, 0, 0])
u0 = np.array([-0.0032115, 0.6792])
y0 = np.array([0, -1524, 0, 62.3866, 0, 0, 0, 0, 62.3866, 1.0557])

t = np.linspace(0,60,10000)
D_u = np.zeros((t.shape[0],2))
D_delta_e = np.zeros_like(t)
for i in range(len(D_delta_e)):
	if t[i] >= 10 and t[i]<=12:
		D_delta_e[i] = 1.0 * np.pi/180
	if t[i] > 12 and t[i]<=14:
		D_delta_e[i] = -1.0 * np.pi/180
D_delta_t = np.zeros_like(t)
D_u[:,0] = D_delta_e
D_u[:,1] = D_delta_t

# Simulation
yout, tout, xout = lsim(sys, D_u, t)

# Parsing results
Elevator_deg 	= (u0[0] + D_delta_e) * 180/np.pi
Throtlle 		= (u0[1] + D_delta_t)
XI_m 			= (y0[0] + yout[:,0])
Alt_ft 			= (y0[1] + yout[:,1]) * (-3.28084)
Theta_deg 		= (y0[2] + yout[:,2]) * 180/np.pi
u_mps 			= (y0[3] + yout[:,3])
w_mps 			= (y0[4] + yout[:,4])
q_degps 		= (y0[5] + yout[:,5]) * 180/np.pi
Alpha_deg 		= (y0[6] + yout[:,6]) * 180/np.pi
Gamma_deg 		= (y0[7] + yout[:,7]) * 180/np.pi
KTAS 			= (y0[8] + yout[:,8]) * 1.94384
Density_kgpm3 	= (y0[9] + yout[:,9])

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
plt.suptitle('Longitudinal dynamics under elevator doublet')
plt.tight_layout()
plt.show()






