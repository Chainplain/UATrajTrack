from FTslideControl import Under_Traj_Track_Controller as UTTC
import numpy as np

control_4_test = UTTC(0.02)

p_r = np.mat( [-0.1,0.1,0] ).T
p = np.mat( [0,0,0] ).T

v_r = np.mat( [0,0,0] ).T
v = np.mat( [0,0,0] ).T

psi = 3 * np.pi/4
omega_psi = 0


control_4_test.Calc_u_t(p_r, p, v_r, v, psi, omega_psi)

print("Generated Gamma x:", control_4_test.Gamma_xp)
print("Generated Gamma y:", control_4_test.Gamma_yp)
print("Generated Gamma z:", control_4_test.Gamma_zp)

psi =  - 1 * np.pi/4-0.2
control_4_test.Calc_u_t(p_r, p, v_r, v, psi, omega_psi)

print("Generated Gamma x:", control_4_test.Gamma_xp)
print("Generated Gamma y:", control_4_test.Gamma_yp)
print("Generated Gamma z:", control_4_test.Gamma_zp)

psi =  0
p_r = np.mat( [0,0,0] ).T
p = np.mat( [0,0,0] ).T

v_r = np.mat( [1,0,0] ).T
v = np.mat( [0,0,0] ).T

control_4_test.Calc_u_t(p_r, p, v_r, v, psi, omega_psi)

print("Generated Gamma x:", control_4_test.Gamma_xp)
print("Generated Gamma y:", control_4_test.Gamma_yp)
print("Generated Gamma z:", control_4_test.Gamma_zp)

psi =  0
p_r = np.mat( [0,0,0] ).T
p = np.mat( [0,0,0] ).T

v_r = np.mat( [1,0,0] ).T
v = np.mat( [0,0,0] ).T

control_4_test.Calc_u_t(p_r, p, v_r, v, psi, omega_psi)

print("Generated Gamma x:", control_4_test.Gamma_xp)
print("Generated Gamma y:", control_4_test.Gamma_yp)
print("Generated Gamma z:", control_4_test.Gamma_zp)