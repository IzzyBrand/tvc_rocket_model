import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

sin = np.sin
cos = np.cos

# state is [x, y, theta, xdot, ydot, thetadot]

T = 10.        # thrust
L = -1.        # length from CG of rocket to the motor
Dv = 0.1        # the linear drag on the rocket
Dw = 1.     # the rotational drag on the rocket
I = 1.        # the rotational inertia of the rocket
l = -0.5        # the distance from the CG of the rocket to the center of aerodynamic pressure
g = 9.8     # acceleration due to gravity


dt = 0.001
duration = 10
framerate = 20

# a function that returns a thrust angle based on the rocket's state
def control(u):
    return 0

def ddt(u, t):
    x, y, theta, xdot, ydot, thetadot = u

    c = control(u)                  # the control input
    v2 = (xdot**2 + ydot**2)      # the speed squared
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

    linear_drag = abs(v2*sin(theta-phi)*Dv)
    rotational_drag = abs(thetadot)*thetadot*Dw

    xddot = -sin(theta+c)*T + sin(phi)*linear_drag

    yddot = cos(theta+c)*T - cos(phi)*linear_drag - g

    thetaddot = sin(c)*T*L/I - rotational_drag/I + sin(theta-phi)*l*v2*Dv/I

    return np.array([xdot, ydot, thetadot, xddot, yddot, thetaddot])

def animate(x, y, theta):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def update(t):
        t = int(t/(framerate*dt))

        ax.clear()

        plt.xlim(np.min(x)-10, np.max(x)+10)
        plt.ylim(np.min(y)-10, np.max(y)+10)
        plt.gca().set_aspect('equal', adjustable='box')

        size = 10
        arrow = mpatches.Arrow(x[t], y[t],
                               -size*np.sin(theta[t]), size*np.cos(theta[t]), width=size)
        ax.add_patch(arrow)

    ani = FuncAnimation(fig, update, frames=int(len(x)*(framerate*dt)-1), interval=1000./framerate)
    plt.show()


if __name__ == '__main__':

    u0 = np.array([0, 0, 0, 10, 0, 0], dtype=float)
    ts = np.linspace(0, duration, duration/dt)
    us = odeint(ddt, u0, ts)

    x, y, theta, xdot, ydot, thetadot = us.T
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical
    animate(x, y, theta)

    # labels = ['x', 'y', 'theta', 'xdot', 'ydot', 'thetadot']
    # for label, data in zip(labels, us.T):
    #     plt.plot(ts, data, label=label)

    # plt.plot(ts, phi, label='phi')
    # plt.legend()
    # plt.show()
