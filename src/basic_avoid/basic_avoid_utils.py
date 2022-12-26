import numpy as np
from scipy.optimize import fsolve as scipy_fsolve
from src.basic_avoid.basic_avoid_consts import Point, Circle
from typing import Callable


def traj_function(a: Point, b: Point, general_form=False) -> Callable[[float, [float]], float]:
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
    if general_form:
        return lambda x, y: m*x + p - y  # TODO: MIGHT cause some calculus errors one day. It's just a feeling though..
    return lambda x: m*x + p


def circle_gen_eq(center: Point, r: float) -> Callable[[float, float], float]:
    """
    Returns the general form of the equation of a circle
    Tip : general form is an equation equal to  0
    """
    return lambda x, y: np.power(x - center.x, 2) + np.power(y - center.y, 2) - np.power(r, 2)


def angle_towards(src:Point, dst: Point) -> float:
    """
    Returns the angle in radian from one point towards another one point
    """
    return np.arctan2(
        dst.y - src.y,
        dst.x - src.x
    )


def compute_intersections(circle: Circle, line: tuple[Point, Point]) -> tuple[np.ndarray, bool]:
    """
    Using a circle and the source and two distinct points of a line, computes
    the number of crossing points between the circle and the line.
    To do this, it computes and uses the general equation of the given circle : (x - h)² + (y - k)² = r²
    and the general form of the line : m*x + p - y = 0

    Warning : Refactor with care and determination, if you ever dare to
    """

    # Compute straight line trajectory
    # Important : set two_parameterized to True to make it compliant with scipy.optimize.fsolve with circle general form
    line_fc = traj_function(line[0], line[1], general_form=True)

    # Compute circle trajectory
    circle_fc = circle_gen_eq(circle.center, circle.r)

    # Use scipy.optimize.fsolve to get the intersection point(s)
    # This solves the equation circle_fc = line_fc
    # effectively computing the intersection points
    # The function might not find a proper root. We ask for the full output of the function
    # and only grab the returned roots and a special return value.
    #
    # Most readable one-liner I've ever written x)
    roots, _, retval, _ = scipy_fsolve(lambda xy: [line_fc(xy[0], xy[1]), circle_fc(xy[0], xy[1])], np.zeros(2), full_output=True)

    # The special return value is set to 1 if correct roots have been found
    solution_found = retval == 1

    return roots, solution_found
