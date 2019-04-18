import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt

sin = np.sin
cos = np.cos

# state is [x, y, theta, xdot, ydot, thetadot]

T = 0.		# thrust
L = -1.		# length from CG of rocket to the motor
Dv = 0.1		# the linear drag on the rocket
Dw = 1. 	# the rotational drag on the rocket
I = 1.		# the rotational inertia of the rocket
l = -0.5		# the distance from the CG of the rocket to the center of aerodynamic pressure
g = 9.8 	# acceleration due to gravity


# a function that returns a thrust angle based on the rocket's state
def control(u):
	return 0

def ddt(u, t):
	x, y, theta, xdot, ydot, thetadot = u

	c = control(u)				  # the control input
	v2 = (xdot**2 + ydot**2)	  # the speed squared
	phi = np.arctan2(xdot, ydot) # the of the velocity vector away from vertical

	linear_drag = abs(v2*sin(theta-phi)*Dv)
	rotational_drag = abs(thetadot)*thetadot*Dw

	xddot = -sin(theta+c)*T - sin(phi)*linear_drag

	yddot = cos(theta+c)*T - cos(phi)*linear_drag

	thetaddot = sin(c)*T*L/I - rotational_drag/I + sin(theta-phi)*l*v2*Dv/I

	return np.array([xdot, ydot, thetadot, xddot, yddot, thetaddot])


if __name__ == '__main__':
	u0 = np.array([0, 0, 0, 0, 10, -2], dtype=float)
	ts = np.linspace(0, 10, 10000)
	us = odeint(ddt, u0, ts)

	labels = ['x', 'y', 'theta', 'xdot', 'ydot', 'thetadot']


	for label, data in zip(labels, us.T):
		plt.plot(ts, data, label=label)

	x, y, theta, xdot, ydot, thetadot = us.T
	psi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

	plt.plot(ts, psi, label='psi')

	plt.legend()
	plt.show()