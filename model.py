import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

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
    ax.set_xlim(np.min(x)-10, np.max(x)+10)
    ax.set_ylim(np.min(y)-10, np.max(y)+10)
    scat, = ax.plot(x[0], y[0], ls=" ", marker=(3, 0, theta[0]), markersize=20)

    def update(t):
        scat.set_data(x[15*t], y[15*t])
        scat.set_marker((3, 0, np.degrees(theta[10*t])))
        return scat,

    ani = FuncAnimation(fig, update, frames=len((x-1)//10), interval=1)
    plt.show()


if __name__ == '__main__':
    u0 = np.array([0, 0, 0, 10, 0, 0], dtype=float)
    ts = np.linspace(0, 10, 10000)
    us = odeint(ddt, u0, ts)

    labels = ['x', 'y', 'theta', 'xdot', 'ydot', 'thetadot']


    #for label, data in zip(labels, us.T):
    #    plt.plot(ts, data, label=label)

    x, y, theta, xdot, ydot, thetadot = us.T
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical
    animate(x, y, theta)

    #plt.plot(ts, phi, label='phi')
    #plt.legend()
    #plt.show()
