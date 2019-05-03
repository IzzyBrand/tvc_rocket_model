import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

sin = np.sin
cos = np.cos

# state is [x, y, theta, xdot, ydot, thetadot]

# dynamics parameters
# T = 11       # thrust
L = -1.        # length from CG of rocket to the motor
Dv = 0.        # the linear drag on the rocket
Dw = 0.     # the rotational drag on the rocket
I = 1.        # the rotational inertia of the rocket
l = -0        # the distance from the CG of the rocket to the center of aerodynamic pressure
g = 10     # acceleration due to gravity
m = 1

# simulation/rendering parameters
dt = 0.001
duration = 10
framerate = 20
rocket_size = 1

# a function that returns a thrust angle based on the rocket's state
def control(u):
    x, y, theta, xdot, ydot, thetadot = u

    angle_control = theta + thetadot 
    x_control = - 0.05*xdot - 0.01*x
    return angle_control + x_control

    # max_angle = np.pi/4
    # return np.clip(10*theta, -max_angle, max_angle)

def thrust(u):
    x, y, theta, xdot, ydot, thetadot = u
    return np.clip(g - 0.1*y - ydot, 0, 50)

# the rocket dynamics
def ddt(u, t):
    x, y, theta, xdot, ydot, thetadot = u

    T = thrust(u)
    c = control(u)                  # the control input
    v2 = (xdot**2 + ydot**2)      # the speed squared
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

    linear_drag = abs(v2*sin(theta-phi)*Dv)
    rotational_drag = abs(thetadot)*thetadot*Dw

    xddot = (1/m) * (-sin(theta+c)*T + sin(phi)*linear_drag)

    yddot = (1/m) * (cos(theta+c)*T - cos(phi)*linear_drag) - g

    thetaddot = (1/I) * (sin(c)*T*L - rotational_drag + sin(theta-phi)*l*v2*Dv)

    return np.array([xdot, ydot, thetadot, xddot, yddot, thetaddot])

# draw a matplotlib animation of the rocket trajectory
def animate(x, y, theta):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def update(t):
        t = int(t/(framerate*dt))

        ax.clear()

        plt.xlim(np.min(x)-10, np.max(x)+10)
        plt.ylim(np.min(y)-10, np.max(y)+10)
        plt.gca().set_aspect('equal', adjustable='box')


        # rotate theta by 90 degrees because we measure theta=0 from the vertical
        arrow = mpatches.FancyArrow(x[t], y[t], rocket_size*np.cos(theta[t]+np.pi/2), np.sin(theta[t]+np.pi/2),
                                head_width=rocket_size, width=rocket_size)
        ax.add_patch(arrow)

    ani = FuncAnimation(fig, update, frames=int(len(x)*(framerate*dt)-1), interval=1000./framerate)
    plt.show()


if __name__ == '__main__':

    x           = 20
    y           = 50
    theta       = 0
    x_dot       = 0
    y_dot       = 0
    theta_dot   = 0

    u0 = np.array([x, y, theta, x_dot, y_dot, theta_dot], dtype=float)
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
