import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt
import matplotlib.animation as animation
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
mu = -0.1

# simulation/rendering parameters
dt = 0.001
duration = 10
framerate = 20

# a function that returns a thrust angle based on the rocket's state
def control(u):
    x, y, theta, xdot, ydot, thetadot = u

    angle_control =  theta + thetadot
    x_control =  - mu * xdot
    return angle_control + x_control

    # max_angle = np.pi/4
    # return np.clip(10*theta, -max_angle, max_angle)

def thrust(u):
    x, y, theta, xdot, ydot, thetadot = u
    return np.clip(g - ydot, 0, 50)

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
def animate(us):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def update(t):
        t = int(t/(framerate*dt))

        ax.clear()

        plt.xlim(np.min(us[:,0])-10, np.max(us[:,0])+10)
        plt.ylim(np.min(us[:,1])-10, np.max(us[:,1])+10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title('x={}, y={}, theta={}\ndx={}, dy={}, dtheta={}'.format(*us[0]))


        ax.hlines(0, -100, 100, colors=['k'], zorder=-1)
        draw_rocket(ax, us[t])
        ax.add_patch(mpatches.Ellipse([0.,0.],5.,1.))


    ani = FuncAnimation(fig, update, frames=int(us.shape[0]*(framerate*dt)-1), interval=1000./framerate)
    plt.show()

    # Set up formatting for the movie files
    # Writer = animation.writers['ffmpeg']
    # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

    # ani.save('easy_landing.mp4', writer=writer)

def draw_rocket(ax, u):

    rocket_size = 4
    thrust_size = 0.5

    x, y, theta, xdot, ydot, thetadot = u
    # rotate theta by 90 degrees because we measure theta=0 from the vertical
    rocket_arrow = mpatches.FancyArrow(x, y, rocket_size*np.cos(theta+np.pi/2), rocket_size*np.sin(theta+np.pi/2),
                            head_width=rocket_size/4, width=rocket_size/4, color='k')
    ax.add_patch(rocket_arrow)

    c = control(u)
    T = thrust(u)
    thrust_arrow = mpatches.FancyArrow(x, y, T*np.cos(theta+c-np.pi/2)/7., T*np.sin(theta+c-np.pi/2)/7.,
                            head_width=thrust_size, width=thrust_size, color='r')
    ax.add_patch(thrust_arrow)


def plot_state(us, ts):
    plt.rc('text', usetex=True)

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('Position (m), Velocity (m/s)')

    ax2 = ax1.twinx()
    ax2.set_ylabel('Orientation (radians), Angular Velocity (radians/s)')

    plt.title(r'Simulated Rocket State. $\mu$={}'.format(mu))

    labels = ['x', 'y', 'theta', 'xdot', 'ydot', 'thetadot']
    colors = ['r', 'g', 'b', 'r', 'g', 'b']
    alphas = [1, 1, 1, 0.6, 0.6, 0.6]
    for label, data, color, alpha in zip(labels, us.T, colors, alphas):
        plt.plot(ts, data, label=label, c=color, alpha=alpha)

    plt.legend()
    plt.show()

if __name__ == '__main__':

    x           = 0
    y           = 3
    theta       = 0
    x_dot       = 2
    y_dot       = 0
    theta_dot   = 0

    u0 = np.array([x, y, theta, x_dot, y_dot, theta_dot], dtype=float)
    ts = np.linspace(0, duration, duration/dt)
    us = odeint(ddt, u0, ts)


    x, y, theta, xdot, ydot, thetadot = us.T
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

    # animate(us)
    plot_state(us, ts)


