import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# imports for configuration
import sys
import argparse
from config import config, angle_controllers, thrust_controllers
from presets import presets

sin = np.sin
cos = np.cos

# state is [x, y, theta, xdot, ydot, thetadot]

# dynamics parameters
L = config['L']        # length from CG of rocket to the motor
Dv = config['Dv']        # the linear drag on the rocket
Dw = config['Dw']     # the rotational drag on the rocket
I = config['I']        # the rotational inertia of the rocket
l = config['l']        # the distance from the CG of the rocket to the center of aerodynamic pressure
g = config['g']    # acceleration due to gravity
m = config['m']

# simulation/rendering parameters
dt = config['dt']
duration = config['duration']
framerate = config['framerate']

# the rocket dynamics
def ddt(u, t, angle_control, thrust_control):
    x, y, theta, xdot, ydot, thetadot = u

    T = thrust_control(u)
    c = angle_control(u)                 # the control input
    v2 = (xdot**2 + ydot**2)      # the speed squared
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

    linear_drag = abs(v2*sin(theta-phi)*Dv)
    rotational_drag = abs(thetadot)*thetadot*Dw

    xddot = (1/m) * (-sin(theta+c)*T + sin(phi)*linear_drag)

    yddot = (1/m) * (cos(theta+c)*T - cos(phi)*linear_drag) - g

    thetaddot = (1/I) * (sin(c)*T*L - rotational_drag + sin(theta-phi)*l*v2*Dv)

    return np.array([xdot, ydot, thetadot, xddot, yddot, thetaddot])

# draw a matplotlib animation of the rocket trajectory
def animate(us, show_trail):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def update(t):
        t = int(t/(framerate*dt))

        if not show_trail:
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
    # pull control functions from global scope
    global angle_control, thrust_control

    rocket_size = config['rocket_size']
    thrust_size = config['thrust_size']

    x, y, theta, xdot, ydot, thetadot = u

    # rotate theta by 90 degrees because we measure theta=0 from the vertical
    rocket_arrow = mpatches.FancyArrow(x, y, rocket_size*np.cos(theta+np.pi/2),
            rocket_size*np.sin(theta+np.pi/2), head_width=rocket_size/4, width=rocket_size/4,
            color='k')

    ax.add_patch(rocket_arrow)

    c = angle_control(u)
    T = thrust_control(u)
    thrust_arrow = mpatches.FancyArrow(x, y, T*np.cos(theta+c-np.pi/2)/7.,
            T*np.sin(theta+c-np.pi/2)/7., head_width=thrust_size, width=thrust_size, color='r')
    ax.add_patch(thrust_arrow)


def plot_state(us, ts, mu):
    plt.rc('text', usetex=True)

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('Position (m), Velocity (m/s)')

    ax2 = ax1.twinx()
    ax2.set_ylabel('Orientation (radians), Angular Velocity (radians/s)')

    plt.title(r'Simulated Rocket State')

    labels = ['x', 'y', 'theta', 'xdot', 'ydot', 'thetadot']
    colors = ['r', 'g', 'b', 'r', 'g', 'b']
    alphas = [1, 1, 1, 0.6, 0.6, 0.6]
    for label, data, color, alpha in zip(labels, us.T, colors, alphas):
        plt.plot(ts, data, label=label, c=color, alpha=alpha)

    plt.legend()
    plt.show()

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Configure simulation')
    parser.add_argument('--preset', type=str, default=None, choices=presets.keys())
    parser.add_argument('--angle_control', type=str, default='none',
            choices=angle_controllers.keys(),
            help='choose preset function which takes in a state and returns thrust angle')
    parser.add_argument('--thrust_control', type=str, default='none',
            choices=thrust_controllers.keys(),
            help='choose preset function which takes in a state and returns thrust magnitude')
    parser.add_argument('--x',         nargs='?', type=float, const=True, default=0) 
    # default y_0 to 1e-6 to avoid divide by zero errors
    parser.add_argument('--y',         nargs='?', type=float, const=True, default=1e-6) 
    parser.add_argument('--theta',     nargs='?', type=float, const=True, default=0) 
    parser.add_argument('--xdot',     nargs='?', type=float, const=True, default=0) 
    parser.add_argument('--ydot',     nargs='?', type=float, const=True, default=0) 
    parser.add_argument('--thetadot', nargs='?', type=float, const=True, default=0) 
    parser.add_argument('--show_trail', dest='show_trail', action='store_const', const=True,
           default=False)
    return parser.parse_args()


if __name__ == '__main__':
    # parse arguments
    args = parse_args()
    if args.preset == None:
        # get control functions
        angle_control = angle_controllers[args.angle_control]
        thrust_control = thrust_controllers[args.thrust_control]

        # get initial conditions
        x = args.x
        y = args.y
        x_dot = args.xdot
        y_dot = args.ydot

        # switch from degrees to radians
        theta, theta_dot = np.radians(args.theta), np.radians(args.thetadot)
    else:
        preset = presets[args.preset]
        # get control functions
        angle_control = angle_controllers[preset['angle_control']]
        thrust_control = thrust_controllers[preset['thrust_control']]

        # get intial conditions
        x = preset['x']
        y = preset['y']
        x_dot = preset['xdot']
        y_dot = preset['ydot']
        theta = preset['theta']
        theta_dot = preset['thetadot']

    # set up controlled ddt
    def controlled_ddt(u, t): return ddt(u, t, angle_control, thrust_control)

    # run simulation using odeint
    u0 = np.array([x, y, theta, x_dot, y_dot, theta_dot], dtype=float)
    ts = np.linspace(0, duration, int(duration/dt))
    us = odeint(controlled_ddt, u0, ts)

    # unpack state and calculate phi
    x, y, theta, xdot, ydot, thetadot = us.T
    phi = np.arctan2(-xdot, ydot) # angle of the velocity vector from vertical

    # run animation
    animate(us, args.show_trail)

    x, y, theta, xdot, ydot, thetadot = us.T
    phi = np.arctan2(-xdot, ydot) # the of the velocity vector away from vertical

    # animate(us)
    plot_state(us, ts, config['mu'])

