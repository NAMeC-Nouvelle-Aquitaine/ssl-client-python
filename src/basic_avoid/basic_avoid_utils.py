import numpy as np
from scipy.optimize import fsolve as scipy_fsolve
from src.basic_avoid.basic_avoid_consts import Point, Circle
from typing import Callable


def traj_function(a: Point, b: Point, two_parameterized=False) -> Callable[[float, [float]], float]:
    """
    Computes the lambda function describing a straight line
    trajectory from point a to point b

    Special parameter : two_parameterized
        Set to True to return a two-variables lambda function.
        This makes the function usable with scipy.optimize.fsolve alongside a circle equation
        that takes two unknown variables.
        Yeah, that's a weird way to do it, don't kill me :(
    """
    m: float = (b.y - a.y) / (b.x - a.x)

    # Use point a to solve the ordinate of the origin of the function
    # Because y = m*x + p ; y - m*x = p
    p: float = a.y - m * a.x
    if two_parameterized:
        return lambda x, y: m*x + p
    return lambda x: m*x + p


def circle_gen_eq(center: Point, r: float) -> Callable[[float, float], float]:
    return lambda x, y: np.power(x - center.x, 2) + np.power(y - center.y, 2) - np.power(r, 2)


def angle_towards(p: Point) -> float:
    """
    Returns the angle in radian towards a specific point
    """
    return np.arctan2(p.y, p.x)


def compute_intersections(circle: Circle, line: (Point, Point)):
    """
    Using a circle and the source and two distinct points of a line, computes
    the number of crossing points between the circle and the line.
    To do this, it computes and uses the general equation of the given circle : (x - h)² + (y - k)² = r²
    and the cartesian form of the line : y = m*x + p

    Warning : Refactor with care and determination, if you ever dare to
    """

    # Compute straight line trajectory
    # Important : set two_parameterized to True to make it compliant with scipy.optimize.fsolve with circle general form
    line_fc = traj_function(line[0], line[1], two_parameterized=True)

    # Compute circle trajectory
    circle_fc = circle_gen_eq(circle.center, circle.r)

    # Use scipy.optimize.fsolve to get the intersection point(s)
    # Most readable one-liner I've ever written x)
    roots = scipy_fsolve(lambda xy: [line_fc(xy[0], xy[1]), circle_fc(xy[0], xy[1])], np.zeros(2))[0]

    return roots
