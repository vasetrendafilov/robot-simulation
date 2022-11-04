import math
import sympy as sp


def point2(x=0, y=0):
    """Return a 2D point in Cartesian coordinates."""
    return sp.Matrix([x, y])


def hpoint2(x=0, y=0, w=1):
    """Return a 2D point in homogeneous coordinates."""
    return sp.Matrix([x, y, w])


def point3(x=0, y=0, z=0):
    """Return a 3D point in Cartesian coordinates."""
    return sp.Matrix([x, y, z])


def hpoint3(x=0, y=0, z=0, w=1):
    """Return a 3D point in homogeneous coordinates."""
    return sp.Matrix([x, y, z, w])


def unit_vector(vector, coordinates='Cartesian'):
    """
    Normalize the given vector.
    
    :param vector: is vector to be normalized.
    :type vector: Matrix
    ...
    :param coordinates: are coordinates in which the vector is described, 'Cartesian' or 'homogeneous'.
    :type coordinates: string
    ...
    :raise ValueError: if the value of coordinates is invalid.
    ...
    :return: the normalized vector.
    """
    if coordinates == 'Cartesian':
        return vector / math.sqrt(sum(vector.multiply_elementwise(vector)))
    elif coordinates == 'homogeneous':
        vector = vector[:-1, :]
        vector = vector / math.sqrt(sum(vector.multiply_elementwise(vector)))
        return sp.Matrix.vstack(vector, sp.Matrix([0]))
    raise ValueError(f'Invalid coordinates value. Expected "Cartesian" or "homogeneous" but "{coordinates}:was given.')


def rotation3(axis, angle):
    """
    Return a rotation matrix in 3D Cartesian coordinates that represents the rotation by `angle` around `axis`.
    
    :param axis: is the axis of direction. It should be one of ['x', 'y', 'z', 'n', 'o', 'a'].
    :type axis: string
    ...
    :param angle: is the angle of rotation.
    :type angle: sympy expression, sympy symbol or a number
    ...
    raise ValueError: if an invalid axis value is received.
    ...
    :return: the rotation matrix
    """
    if axis in ['x', 'n']:
        return sp.Matrix([
            [1, 0, 0],
            [0, sp.cos(angle), -sp.sin(angle)], 
            [0, sp.sin(angle), sp.cos(angle)]
        ])
    elif axis in ['y', 'o']:
        return sp.Matrix([
            [sp.cos(angle), 0, sp.sin(angle)],
            [0, 1, 0],
            [-sp.sin(angle), 0, sp.cos(angle)]
        ])
    elif axis in ['z', 'a']:
        return sp.Matrix([
            [sp.cos(angle), -sp.sin(angle), 0], 
            [sp.sin(angle), sp.cos(angle), 0], 
            [0, 0, 1]
        ])
    else:
        raise ValueError(f'Expected one of [x, y, z, n, o, a] but received {axis}.')
        

def hrotation3(axis, angle):
    """
    Return a transformation matrix in 3D homogeneous coordinates that represents the rotation by `angle` around `axis`.
    
    :param axis: is the axis of rotation. It should be one of ['x', 'y', 'z', 'n', 'o', 'a'].
    :type axis: string
    ...
    :param angle: is the angle of rotation.
    :type angle: sympy expression, sympy symbol or a number
    ...
    raise ValueError: if an invalid axis value is received.
    ...
    :return: the rotation matrix
    """
    if axis in ['x', 'n']:
        return sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(angle), -sp.sin(angle), 0], 
            [0, sp.sin(angle), sp.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis in ['y', 'o']:
        return sp.Matrix([
            [sp.cos(angle), 0, sp.sin(angle), 0],
            [0, 1, 0, 0],
            [-sp.sin(angle), 0, sp.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis in ['z', 'a']:
        return sp.Matrix([
            [sp.cos(angle), -sp.sin(angle), 0, 0], 
            [sp.sin(angle), sp.cos(angle), 0, 0], 
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        raise ValueError(f'Expected one of [x, y, z, n, o, a] but received {axis}.')

        
def rotation2(angle):
    """Return a rotation matrix in 2D Cartesian coordinates that represents the rotation by the given angle."""
    return sp.Matrix([
        [sp.cos(angle), -sp.sin(angle)], 
        [sp.sin(angle), sp.cos(angle)]
    ])


def hrotation2(angle):
    """Return a rotation matrix in 2D homogeneous coordinates that represents the rotation by the given angle."""
    return sp.Matrix([
        [sp.cos(angle), -sp.sin(angle), 0], 
        [sp.sin(angle), sp.cos(angle), 0], 
        [0, 0, 1]
    ])


def htranslation3(x=0, y=0, z=0):
    """Return transformation matrix in 3D homogeneous coordinates with embedded translation."""
    return sp.Matrix([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])


def htranslation2(x=0, y=0):
    """Return transformation matrix in 2D homogeneous coordinates with embedded translation."""
    return sp.Matrix([[1, 0, x],
                      [0, 1, y],
                      [0, 0, 1]])
        

def is_right_hand_coordinate_system3(pose):
    """Checks whether the given pose follows the right-hand rule."""
    n, o, a = pose[:3, 0], pose[:3, 1], pose[:3, 2]
    return n.dot(n).simplify() == 1 and o.dot(o).simplify() == 1 and a.dot(a).simplify() == 1 and sp.simplify(n.cross(o)) == a


def hpose3(x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1), t=(0, 0, 0)):
    """Return a pose in 3D homogeneous coordinates where the arguments are the new frame's axes and translation vector."""
    return sp.Matrix.vstack(sp.Matrix([tuple(x), tuple(y), tuple(z), tuple(t)]).T, sp.Matrix([0, 0, 0, 1]).T)


def euler_angles(pose, sequence):
    """
    Return the euler angles for the given rotation.
    
    :param pose: is a rotation or transformation matrix.
    :type pose: Matrix
    ...
    :param sequence: is the euler angles configuration.
    :type sequence: string 
    ...
    raise ValueError: if an invalid or unsupported euler angle sequence is received.
    ...
    :return: the euler angles.
    """
#     Ова се дел од Ојлеровите агли.
#     За останатите оди на https://www.geometrictools.com/Documentation/EulerAngles.pdf
    sequence = sequence.lower()
    if sequence == 'zyz':
        if pose[2, 2] == 1:
            theta_1, theta_2, theta_3 = sp.atan2(pose[1, 0], pose[1, 1]), 0, 0
        elif pose[2, 2] == -1:
            theta_1, theta_2, theta_3 = -sp.atan2(pose[1, 0], pose[1, 1]), sp.pi, 0
        else:
            theta_1 = sp.atan2(pose[1, 2], pose[0, 2])
            theta_2 = sp.acos(pose[2, 2])
            theta_3 = sp.atan2(pose[2, 1], -pose[2, 0])
        return sp.Matrix([theta_1, theta_2, theta_3])
    elif sequence == 'zyx':
        if pose[2, 0] == 1:
            theta_1, theta_2, theta_3 = sp.atan2(-pose[1, 2], pose[1, 1]), -sp.pi/2, 0
        elif pose[2, 0] == -1:
            theta_1, theta_2, theta_3 = -sp.atan2(-pose[1, 2], pose[1, 1]), sp.pi/2, 0
        else:
            theta_1 = sp.atan2(pose[1, 0], pose[0, 0])
            theta_2 = sp.asin(-pose[2, 0])
            theta_3 = sp.atan2(pose[2, 1], pose[2, 2])
        return sp.Matrix([theta_1, theta_2, theta_3])
    elif sequence == 'xyz':
        if pose[0, 2] == 1:
            theta_1, theta_2, theta_3 = sp.atan2(pose[1, 0], pose[1, 1]), sp.pi/2, 0
        elif pose[0, 2] == -1:
            theta_1, theta_2, theta_3 = -sp.atan2(pose[1, 0], pose[1, 1]), -sp.pi/2, 0
        else:
            theta_1 = sp.atan2(-pose[1, 2], pose[2, 2])
            theta_2 = sp.asin(pose[0, 2])
            theta_3 = sp.atan2(-pose[0, 1], pose[0, 0])
        return sp.Matrix([theta_1, theta_2, theta_3])
    elif sequence == 'xzy':
        if pose[0, 1] == 1:
            theta_1, theta_2, theta_3 = sp.atan2(-pose[2, 0], pose[2, 2]), -sp.pi/2, 0
        elif pose[0, 1] == -1:
            theta_1, theta_2, theta_3 = -sp.atan2(-pose[2, 0], pose[2, 2]), sp.pi/2, 0
        else:
            theta_1 = sp.atan2(pose[2, 1], pose[1, 1])
            theta_2 = sp.asin(-pose[0, 1])
            theta_3 = sp.atan2(pose[0, 2], pose[0, 0])
        return sp.Matrix([theta_1, theta_2, theta_3])
    raise ValueError(f'Invalid or unsupported euler angle sequence {sequence}.')


def rotation_matrix_from_euler_angles(euler_angles, sequence):
    """
    Return a transformation matrix for the given euler angles.
    
    :param euler_angles: are the euler angles.
    :type euler_angles: Matrix
    ...
    :param sequence: is the euler angles configuration.
    :type sequence: string 
    ...
    raise ValueError: if an invalid or unsupported euler angle sequence is received.
    ...
    :return: a transformation matrix for the given euler angles.
    """
    sequence = sequence.lower()
    if not all([char in 'xyz' for char in sequence]) or len(sequence) != 3:
        raise ValueError(f'Invalid or unsupported euler angle sequence {sequence}.')
    T1 = hrotation3(sequence[0], euler_angles[0])
    T2 = hrotation3(sequence[1], euler_angles[1])
    T3 = hrotation3(sequence[2], euler_angles[2])
    return T1 * T2 * T3


def lagrangian(L, jvars):
    """ 
    Implementation of Lagrange's equations. 
    
    :param L: is the Lagrangian.
    :type L: Matrix
    ...
    :param jvars: is a list of all variables used in the Lagrangian.
    :type jvars: list or Matrix
    ...
    :return: the generalized forces equations.
    """
    jvars = sp.Matrix(jvars)
    return sp.simplify(L.diff(jvars.diff('t')).diff('t') - L.diff(jvars))


def dynamic_model_with_4_matrices(L, jvars):
    """ 
    Create the dynamic model through implementation of Lagrange's equations as by the book.
    
    :param L: is the Lagrangian.
    :type L: Matrix
    ...
    :param jvars: is a list of all variables used in the Lagrangian.
    :type jvars: list or Matrix
    ...
    :return: the 4 matrices that create the dynamic model.
    """
    import itertools
    from sympy.core.add import Add
    
    n = len(jvars)
    M, Cf, Ck, G = sp.zeros(n), sp.zeros(n), sp.zeros(n, n*(n-1)//2), sp.zeros(n, 1)
    jvars = sp.Matrix(jvars)
    v_jvars, a_jvars = jvars.diff('t'), jvars.diff('t', 2)
    v_jvars_squared = sp.matrix_multiply_elementwise(v_jvars, v_jvars)
    Ck_vars = [var[0] * var[1] for var in itertools.combinations(v_jvars, 2)]
    for row, eq in enumerate(lagrangian(L, jvars).expand()):
        args = eq.args if eq.func is Add else [eq]
        M_args = [arg for arg in args if arg.has(*a_jvars)]
        M[row, :] = [[sum([arg for arg in args if arg.has(var)]) for var in a_jvars]]
        Cf[row, :] = [[sum([arg for arg in args if arg.has(var)]) for var in v_jvars_squared]]
        if n > 1:
            Ck[row, :] = [[sum([arg for arg in args if arg.has(var)]) for var in Ck_vars]]
        G[row, :] = [sum([arg for arg in args if not arg.has(*a_jvars) and not arg.has(*v_jvars)])]
    return sp.simplify(M), sp.simplify(Cf), sp.simplify(Ck), sp.simplify(G)


def dynamic_model_with_3_matrices(L, jvars):
    """ 
    Create the dynamic model through implementation of Lagrange's equations as by the book.
    
    :param L: is the Lagrangian.
    :type L: Matrix
    ...
    :param jvars: is a list of all variables used in the Lagrangian.
    :type jvars: list or Matrix
    ...
    :return: the 3 matrices that create the dynamic model.
    """
    import itertools
    from sympy.core.add import Add
    
    n = len(jvars)
    M, C, G = sp.zeros(n), sp.zeros(n, n*(n+1)//2), sp.zeros(n, 1)
    jvars = sp.Matrix(jvars)
    v_jvars, a_jvars = jvars.diff('t'), jvars.diff('t', 2)
    v_jvars_squared = sp.matrix_multiply_elementwise(v_jvars, v_jvars)
    C_vars = [var[0] * var[1] for var in itertools.combinations_with_replacement(v_jvars, 2)]
    for row, eq in enumerate(lagrangian(L, jvars).expand()):
        args = eq.args if eq.func is Add else [eq]
        M_args = [arg for arg in args if arg.has(*a_jvars)]
        M[row, :] = [[sum([arg for arg in args if arg.has(var)]) for var in a_jvars]]
        C[row, :] = [[sum([arg for arg in args if arg.has(var)]) for var in C_vars]]
        G[row, :] = [sum([arg for arg in args if not arg.has(*a_jvars) and not arg.has(*v_jvars)])]
    return sp.simplify(M), sp.simplify(C), sp.simplify(G)


def force_transformation_matrix(pose):
    """ Return the force transformation matrix for the given pose."""
    R, P = pose[:3, :3], pose[:3, 3]
    return sp.Matrix.vstack(
        sp.Matrix.hstack(R, sp.zeros(3)),
        sp.Matrix.hstack(delta3(*P)*R, R))


def dh_joint_to_joint(theta, d, a, alpha):
    """ Get the DH model elementary matrix consisted of theta, d, a, apha parameters. """
    return hrotation3('z', theta) * htranslation3(x=a, z=d) * hrotation3('x', alpha)


def frame_lines(pose, line_length=1):
    pose = pose.evalf()
    t = pose[:3, -1]
    line_n = sp.Matrix.hstack(t, t + line_length * pose[:3, 0])
    line_o = sp.Matrix.hstack(t, t + line_length * pose[:3, 1])
    line_a = sp.Matrix.hstack(t, t + line_length * pose[:3, 2])
    return line_n, line_o, line_a


def delta3(dRx=0, dRy=0, dRz=0):
    """
    Return the differential operator for rotation matrix in 3D.
    
    :params dRx, dRy, dRz: are the differential rotations.
    :type dRx, dRy, dRz: Symbol or number
    ...
    :return: the differential operator for rotation matrix in 3D.
    """
    return sp.Matrix([
        [0, -dRz, dRy],
        [dRz, 0, -dRx],
        [-dRy, dRx, 0]
    ])


def hdelta3(dRx=0, dRy=0, dRz=0, dx=0, dy=0, dz=0):
    """
    Return the differential operator for homogeneous matrix in 3D.
    
    :params dRx, dRy, dRz: are the differential rotations.
    :type dRx, dRy, dRz: Symbol or number
    ...
    :params dx, dy, dz: are the differential translations.
    :type dx, dy, dz: Symbol or number 
    ...
    :return: the differential operator for homogeneous matrix in 3D.
    """
    return sp.Matrix([
        [0, -dRz, dRy, dx],
        [dRz, 0, -dRx, dy],
        [-dRy, dRx, 0, dz],
        [0, 0, 0, 0],
    ])


def extract_hdelta3(delta):
    """
    Return a vector consisted of differential rotations and translations.
    The vector elements are [δx, δy, δz, dx, dy, dz].
    
    :param delta: is the differential operator.
    :type delta: Matrix
    ...
    raise ValueError: if an invalid delta operator is received.
    ...
    :return: the vector of differential rotations and translations.
    """
    if delta[2, 1] != -delta[1, 2] or delta[2, 1] != -delta[1, 2] or delta[2, 1] != -delta[1, 2]:
        raise ValueError('delta is not a valid differential operator.')
    dRx, dRy, dRz = delta[2, 1], delta[0, 2], delta[1, 0]
    dx, dy, dz = delta[0, 3], delta[1, 3], delta[2, 3]
    return sp.Matrix([dRx, dRy, dRz, dx, dy, dz])


def polynomial(n, char='c'):
    """ Create an n-degree polynomial with custom symbols / coefficients. """
    symbols = [sp.symbols(f'{char}{i}') for i in range(n+1)]
    p = sum([sp.symbols('t')**i * symbols[i] for i in range(n+1)])
    return p, symbols
    

def trajectory_polynomial_3(ti, tf, xi, xf, vi, vf):
    """
    Create a trajectory with 3th degree polynomial.
    
    :param ti: is the initial time.
    :type ti: number
    ...
    :param tf: is the final time.
    :type tf: number
    ...
    :param xi: is the initial position.
    :type xi: number
    ...
    :param xf: is the final position.
    :type xf: number
    ...
    :param vi: is the initial velocity.
    :type vi: number
    ...
    :param vf: is the final velocity.
    :type vf: number
    ...
    :return: the coefficients and position equation.
    """
    theta, symbols = polynomial(3)
    v_theta = theta.diff('t')
    eq_xi = theta.subs('t', ti) - xi
    eq_xf = theta.subs('t', tf) - xf
    eq_vi = v_theta.subs('t', ti) - vi
    eq_vf = v_theta.subs('t', tf) - vf
    solution = sp.linsolve([eq_xi, eq_xf, eq_vi, eq_vf], symbols)
    for sub in zip(symbols, solution.args[0]):
        theta =theta.subs(*sub)
    return theta, solution


def trajectory_polynomial_5(ti, tf, xi, xf, vi, vf, ai, af):
    """
    Create a trajectory with 5th degree polynomial.
    
    :param ti: is the initial time.
    :type ti: number
    ...
    :param tf: is the final time.
    :type tf: number
    ...
    :param xi: is the initial position.
    :type xi: number
    ...
    :param xf: is the final position.
    :type xf: number
    ...
    :param vi: is the initial velocity.
    :type vi: number
    ...
    :param vf: is the final velocity.
    :type vf: number
    ...
    :param ai: is the initial acceleration.
    :type ai: number
    ...
    :param af: is the final acceleration.
    :type af: number
    ...
    :return: the coefficients and position equation.
    """
    theta, symbols = polynomial(5)
    v_theta = theta.diff('t')
    a_theta = theta.diff('t', 2)
    eq_xi = theta.subs('t', ti) - xi
    eq_xf = theta.subs('t', tf) - xf
    eq_vi = v_theta.subs('t', ti) - vi
    eq_vf = v_theta.subs('t', tf) - vf
    eq_ai = a_theta.subs('t', ti) - ai
    eq_af = a_theta.subs('t', tf) - af
    solution = sp.linsolve([eq_xi, eq_xf, eq_vi, eq_vf, eq_ai, eq_af], symbols)
    for sub in zip(symbols, solution.args[0]):
        theta =theta.subs(*sub)
    return theta, solution
    
    
def trajectory_linear_parabolic_segments(ti, tf, xi, xf, w=None, tb=None):
    """
    Create a linear trajectory with parabolic end-segments.
    
    Note: Only one argument of w and tb is enough to get numerical solution.
    
    :param ti: is the initial time.
    :type ti: number
    ...
    :param tf: is the final time.
    :type tf: number
    ...
    :param xi: is the initial position.
    :type xi: number
    ...
    :param xf: is the final position.
    :type xf: number
    ...
    :param w: is the constant velocity in the linear segment.
    :type w: number
    ...
    :param tb: is the duration of each parabolic segment.
    :type tb: number
    ...
    :return: the coefficients w and tb and position equation of each segment.
    """
    if w and tb is None:
        tb = (xi - xf + w*tf) / w
    elif w is None and tb:
        w = (xi - xf) / (tb - tf)
    elif w is None and tb is None:
        w, tb = sp.symbols('t_b, w')
    t = sp.symbols('t')
    theta_s1 = xi + w / (2*tb) * t**2
    theta_s2 = theta_s1.subs(t, tb) - w*tb + w*t
    theta_s3 = xf - w / (2*tb) * (tf-t)**2
    return theta_s1, theta_s2, theta_s3, w, tb