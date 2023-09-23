import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from readTraj import traj_reader as TR
# Load the .mat file
mat_data = scipy.io.loadmat('way_point.mat')

aa = TR('way_point.mat')
Time = np.linspace(0, 11.99, 200)
y_vel_here = []
for tau in Time:
    y_vel_here.append(aa.get_y_vel(tau))
plt.plot(Time, y_vel_here)
plt.title('Plot of v(x)')
plt.show()

# Access the variables in the .mat file
# Coefs = mat_data['coef'].flatten().tolist()

# Timepersecond = mat_data['T'].flatten().tolist()

# print('Timepersecond:',Timepersecond)
# print('Coefs:', Coefs[0:7])

# One_seg_coef_num = 7 * 3
# funs = []
# x1 = np.poly1d(Coefs[0:7])
# funs.append(x1)

# x2 = np.poly1d(Coefs[One_seg_coef_num + 0 :One_seg_coef_num + 7])
# funs.append(x2)

# x3 = np.poly1d(Coefs[One_seg_coef_num * 2 + 0 :One_seg_coef_num * 2 + 7])
# funs.append(x3)

# tau = np.linspace(0, Timepersecond, 100)

# X1 = funs[0](tau)
# X2 = funs[1](tau)
# X3 = funs[2](tau)


# plt.plot(tau, X1)
# plt.plot(tau + Timepersecond[0], X2)
# plt.plot(tau + Timepersecond[0]*2, X3)

# plt.title('Plot of p(x)')
# plt.show()

