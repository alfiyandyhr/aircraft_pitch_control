# Written by Alfiyandy Hariansyah, HKUST -- December 2024
from flightgear_python.fg_if import FDMConnection
from control import ss, tf, interconnect, summing_junction
from control.matlab import lsim, step
from coord_transform import flat2lla
import numpy as np
import time

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
y0 = np.array([0, -1524, 0, 62.3866, 0, 0, 0, 0, 62.3866, 1.0557])

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
TF = 10
fps = 30
t = np.arange(0,TF,1/fps)
ref_theta = 0.2 * np.ones_like(t) # 0.2 rad
yout, tout, xout = lsim(cl_sys, ref_theta, t)

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

# Change coordinates: from flat planet to LLA
lat_deg, lon_deg, alt_m = flat2lla(XI_m, 0, ZI_m, 22.3272, 113.948, 0, 250.8)
lat_rad = lat_deg * np.pi/180
lon_rad = lon_deg * np.pi/180

i = 0
def fdm_callback(fdm_data, event_pipe):
	global i
	fdm_data.lat_rad 	= lat_rad[i]
	fdm_data.lon_rad 	= lon_rad[i]
	fdm_data.alt_m 		= alt_m[i]
	fdm_data.phi_rad 	= 0
	fdm_data.theta_rad 	= Theta_rad[i]
	fdm_data.psi_rad 	= 250.8 * np.pi/180
	fdm_data.elevator 	= Elevator_rad[i]
	i += 1
	if i == len(tout): i = 0 # rewind
	if tout[i] in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
		print(f't = {round(tout[i],2)}, Pitch angle = {round(fdm_data.theta_rad * 180/np.pi,2)} deg, Altitude = {round(fdm_data.alt_m * 3.28084,2)} ft')
	return fdm_data  # return the whole structure

"""
Start FlightGear with `--native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp`
(you probably also want `--fdm=null` and `--max-fps=30` to stop the simulation fighting with
these external commands)
"""
fdm_conn = FDMConnection()
fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
fdm_conn.connect_tx('localhost', 5502)
fdm_conn.start()  # Start the FDM RX/TX loop
while True:
	time.sleep(0.01)