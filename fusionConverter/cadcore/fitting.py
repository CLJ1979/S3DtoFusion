import math
import numpy as np
from geomdl import BSpline, helpers, linalg, utilities

# def compute_knot_vector_w_derivative(degree, num_points, params):
#     """ Computes a knot vector from the parameter list using averaging method.

#     Please refer to the Equation 9.9 on The NURBS Book (2nd Edition), pp.370 for details.

#     :param degree: degree
#     :type degree: int
#     :param num_points: number of data points
#     :type num_points: int
#     :param params: list of parameters, :math:`\\overline{u}_{k}`
#     :type params: list, tuple
#     :return: knot vector
#     :rtype: list
#     """
#     # Start knot vector
#     kv = [0.0 for _ in range(degree + 1)]

#     # Use averaging method (Eqn 9.9) to compute internal knots in the knot vector
#     for i in range(num_points - degree + 1):
#         temp_kv = (1.0 / degree) * sum([params[j] for j in range(i, i + degree)])
#         kv.append(temp_kv)

#     # End knot vector
#     kv += [1.0 for _ in range(degree + 1)]

#     return kv


def compute_params_curve(points, centripetal=False):
    """ Computes :math:`\\overline{u}_{k}` for curves.

    Please refer to the Equations 9.4 and 9.5 for chord length parametrization, and Equation 9.6 for centripetal method
    on The NURBS Book (2nd Edition), pp.364-365.

    :param points: data points
    :type points: list, tuple
    :param centripetal: activates centripetal parametrization method
    :type centripetal: bool
    :return: parameter array, :math:`\\overline{u}_{k}`
    :rtype: list
    """
    if not isinstance(points, (list, tuple)):
        raise TypeError("Data points must be a list or a tuple")

    # Length of the points array
    num_points = len(points)

    # Calculate chord lengths
    cds = [0.0 for _ in range(num_points + 1)]
    cds[-1] = 1.0
    for i in range(1, num_points):
        distance = linalg.point_distance(points[i], points[i - 1])
        cds[i] = math.sqrt(distance) if centripetal else distance

    # Find the total chord length
    d = sum(cds[1:-1])

    # Divide individual chord lengths by the total chord length
    uk = [0.0 for _ in range(num_points)]
    for i in range(num_points):
        uk[i] = sum(cds[0:i + 1]) / d

    return uk

def compute_cubic_knot_vector_w_derivative(params):
    """ Computes a knot vector from the parameter list using averaging method.

    Please refer to the Equation 9.9 on The NURBS Book (2nd Edition), pp.370 for details.

    :param params: list of parameters, :math:`\\overline{u}_{k}`
    :type params: list, tuple
    :return: knot vector
    :rtype: list
    """
    degree = 3

    # Start knot vector
    kv = [0.0 for _ in range(degree + 1)]

    # For cubic splines, interpolation of the data points occurs at the knots
    kv += params[1:-1]

    # End knot vector
    kv += [1.0 for _ in range(degree + 1)]

    return kv

def _solve_tridiagonal(knot_vector, points, derivatives, uk):
    """Solve tridiagonal system for C2 cubic splines.

    See Algorithm A9.2 in The NURBS Book (2nd Edition), pp.373 for details.
    """
    dim = len(points[0])
    num_points = len(points)
    n = num_points - 1

    points = np.array(points)
    derivatives = np.array(derivatives)

    # Eq. 9.14
    P = np.zeros(shape=(num_points+2, dim))
    P[0] = points[0]
    P[1] = P[0] + uk[4]/3 * derivatives[0]

    # Eq. 9.15
    P[-1] = points[-1]
    P[-2] = P[-1] - (1 - uk[1])/3 * derivatives[-1]
    
    # Initialise local lists
    R = np.zeros(shape=(num_points, dim))
    dd = np.zeros(shape=num_points)

    R[3:n] = points[2:n-1]
    
    abc = helpers.basis_function(degree=3, knot_vector=knot_vector, span=4, knot=knot_vector[4])
    den = abc[1]
    P[2] = (points[1] - abc[0]*P[1]) / den

    for i in range(3, n):
        dd[i] = abc[2] / den
        abc = helpers.basis_function(degree=3, knot_vector=knot_vector, span=i+2, knot=knot_vector[i+2])
        den = abc[1] - abc[0] * dd[i]
        P[i] = (R[i] - abc[0] * P[i-1]) / den

    dd[n] = abc[2] / den
    abc = helpers.basis_function(degree=3, knot_vector=knot_vector, span=n+2, knot=knot_vector[n+2])
    den = abc[1] - abc[0] * dd[n]
    P[n] = (points[n-1] - abc[2] * P[n+1] - abc[0] * P[n-1]) / den

    for i in range(n-1, 1, -1):
        P[i] = P[i] - dd[i+1] * P[i+1]

    return P.tolist()

def interpolate_cubic_spline_curve_w_derivative(points, derivatives, **kwargs):
    """ Curve interpolation through the data points.

    Please refer to Algorithm A9.1 on The NURBS Book (2nd Edition), pp.369-370 for details.

    Keyword Arguments:
        * ``centripetal``: activates centripetal parametrization method. *Default: False*

    :param points: data points
    :type points: list, tuple
    :param degree: degree of the output parametric curve
    :return: interpolated B-Spline curve
    :rtype: BSpline.Curve
    """
    degree = 3 # Cubic

    # Keyword arguments
    use_centripetal = kwargs.get('centripetal', False)

    # Number of control points
    # num_points = len(points)

    # Get uk
    uk = compute_params_curve(points, use_centripetal)

    # Compute knot vector
    # kv = compute_knot_vector_w_derivative(degree, num_points, uk)
    kv = compute_cubic_knot_vector_w_derivative(uk)

    # Do global interpolation
    # matrix_a = _build_coeff_matrix(degree, kv, uk, points)
    # ctrlpts = linalg.lu_solve(matrix_a, points)
    ctrlpts = _solve_tridiagonal(kv, points, derivatives, uk)

    # Generate B-spline curve
    curve = BSpline.Curve()
    curve.degree = degree
    curve.ctrlpts = ctrlpts
    curve.knotvector = kv

    return curve

#### Local interpolation method below here ####

def _compute_tangents(points, smooth_corners=True):
    dim = len(points[0])
    num_points = len(points)
    n = num_points - 1

    # Compute tangents (eqs. 9.29, 9.31 and 9.33)
    q_k = np.zeros(shape=(n + 4, 3))  # k = -1, 0, ..., n+1, n+2
    q_k[2:n+2,:dim] = points[1:] - points[:-1]

    # Eq. 9.33
    q_k[1] = 2*q_k[2] - q_k[3] # k = 0
    q_k[0] = 2*q_k[1] - q_k[2] # k = -1
    q_k[n+2] = 2*q_k[n+1] - q_k[n] # k = n+1
    q_k[n+3] = 2*q_k[n+2] - q_k[n+1] # k = n+2

    # print(q_k.tolist())

    # Eq. 9.31
    q_km1_x_q_k = np.linalg.norm(np.cross(q_k[:n+1], q_k[1:n+2]), axis=1, ord=1)
    q_kp1_x_q_kp2 = np.linalg.norm(np.cross(q_k[2:n+3], q_k[3:]), axis=1, ord=1)
    denominator = q_km1_x_q_k + q_kp1_x_q_kp2
    # print(denominator)
    # print(q_km1_x_q_k)
    mask_collinear_points = denominator < 1e-6
    # print(mask_collinear_points)
    denominator[mask_collinear_points] = 2 if smooth_corners else 1
    q_km1_x_q_k[mask_collinear_points] = 1
    # print(denominator)
    alpha_k = q_km1_x_q_k / denominator

    # print(alpha_k.tolist())

    V_k = (1 - alpha_k[:, np.newaxis])*q_k[1:n+2] + alpha_k[:, np.newaxis]*q_k[2:n+3]
    # print(V_k)
    V_k_magnitude = np.linalg.norm(V_k, axis=1, ord=1)
    T_k = V_k / V_k_magnitude[:, np.newaxis]
    # print(T_k)

    return T_k[:, :dim]

def interpolate_local_cubic_curves(points):
    dim = len(points[0])
    num_points = len(points)
    n = num_points - 1

    points = np.array(points)

    T_k = _compute_tangents(points)

    # Compute control points and u_k
    CTRLPTS = np.zeros(shape=(n, 4, dim))
    # Eq. 9.51
    CTRLPTS[:, 0] = points[:-1]
    CTRLPTS[:, 3] = points[1:]

    u_k = np.zeros(n+1)

    for k in range(n):
        # 1) Compute alpha (eq. 9.50)
        a = 16 - np.linalg.norm(T_k[k] + T_k[k+1], ord=1)**2
        b = 12 * np.dot(points[k+1] - points[k], T_k[k] + T_k[k+1])
        c = -36 * np.linalg.norm(points[k+1] - points[k], ord=1)**2

        alpha = max(np.roots([a, b, c]))

        # Compute P_k,1 and P_k,2 (eq. 9.47)
        CTRLPTS[k, 1] = CTRLPTS[k, 0] + 1/3 * alpha * T_k[k]
        CTRLPTS[k, 2] = CTRLPTS[k, 3] - 1/3 * alpha * T_k[k+1]

        # 2) Compute uk (eq. 9.52)
        u_k[k+1] = u_k[k] + 3 * np.linalg.norm(CTRLPTS[k, 1] - CTRLPTS[k, 0], ord=1)

    # Compute knot vector (eq. 9.54)
    kv = [0] * 4 + [param/u_k[-1] for param in u_k[1:-1] for _ in range(2)] + [1] * 4

    # Eq. 9.53
    ctrlpts = [points[0].tolist()] + np.reshape(CTRLPTS[:, 1:3, :], (n*2, dim)).tolist() + [points[-1].tolist()]

    # # Generate B-spline curve
    # curve = BSpline.Curve()
    # curve.degree = 3
    # curve.ctrlpts = ctrlpts
    # curve.knotvector = kv

    # return curve
        
    curves = []
    for k in range(n):
        curve = BSpline.Curve()
        curve.degree = 3
        curve.ctrlpts = CTRLPTS[k, :].tolist()
        curve.knotvector = utilities.generate_knot_vector(3, 4)
        curves.append(curve)
        
    return curves


