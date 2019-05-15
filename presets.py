presets = {
    'stable': {
        'angle_control': 'stable',
        'thrust_control': 'stable',
        'x': 0,
        'y': 0,
        'xdot': 0.1,
        'ydot': 0,
        'theta': 0,
        'thetadot': 0
        },
    'unstable': {
        'angle_control': 'unstable',
        'thrust_control': 'stable',
        'x': 0,
        'y': 0,
        'xdot': 0.1,
        'ydot': 0,
        'theta': 0,
        'thetadot': 0
        },
    'ballistic': {
        'angle_control': 'none',
        'thrust_control': 'none',
        'x': 0,
        'y': 0,
        'xdot': 5,
        'ydot': 30,
        'theta': 0,
        'thetadot': 2
        },
    }