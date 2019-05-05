import numpy as np


config = {
    # physical constants
    "L"  : -1., 
    "Dv" : 0.,
    "Dw" : 0.,
    "I"  : 1.,
    "l"  : 0.,
    "g"  : 10.,
    "m"  : 1.,

    # simulation/rendering parameters
    "dt" : 0.001,
    "duration" : 10,
    "framerate" : 20,

    # animation constants
    "rocket_size" : 4,
    "thrust_size" : 0.5
}

def angle_p(u):
    x, y, theta, xdot, ydot, thetadot = u

    angle_control = theta
    x_control = -0.01*x

    return angle_control + x_control

def angle_pd(u):
    x, y, theta, xdot, ydot, thetadot = u

    angle_control = theta + thetadot
    x_control = -0.05*xdot - 0.01*x

    return angle_control + x_control

def thrust_p(u):
    x, y, theta, xdot, ydot, thetadot = u
    
    min_thrust = 0
    max_thrust = 50
    thrust_control = np.clip(g - 0.1*y, min_thrust, max_thrust)

    return thrust_control

def thrust_pd(u):
    x, y, theta, xdot, ydot, thetadot = u
    
    min_thrust = 0
    max_thrust = 50
    thrust_control = np.clip(config['g'] - 0.1*y - 10*ydot, min_thrust, max_thrust)

    return thrust_control

def thrust_non_linear(u):
    x, y, theta, xdot, ydot, thetadot = u

    min_thrust = 0
    max_thrust = 50
    thrust_control = np.clip(config['g'] - 0.1*y - 10*ydot/y, min_thrust, max_thrust)

    return thrust_control

def no_control(u):
    return 0

angle_controllers = {
    'p': angle_p,
    'pd': angle_pd,
    'none': no_control
}

thrust_controllers = {
    'p': thrust_p,
    'pd': thrust_pd,
    'non-linear': thrust_non_linear,
    'none': no_control
}