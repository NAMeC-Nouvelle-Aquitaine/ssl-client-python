import numpy as np
from scipy.optimize import fsolve as scipy_fsolve

from .basic_avoid_consts import danger_circle_radius, AVOID_DIST_FACTOR_MAX
from .basic_avoid_types import Circle
from typing import Callable
from src.client import ClientRobot


def distance(a: np.array, b: np.array):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


def danger_circle(r: ClientRobot):
    return Circle(
        r.position,
        danger_circle_radius
    )


def traj_function(a: np.array, b: np.array, general_form=False) -> Callable[[float, [float]], float]:
    """
    Computes the lambda function describing a straight line
    trajectory from point a to point b

    Returns :
        A lambda function representing the slope-intercept
        equation form, as y = m * x + p

    Optional parameter : general_form
        Set to True to return the general form of the equation lambda function.
    """
    m: float = -1

    # Can't create a vertical line function
    # Approximating it using a very low slope coefficient when both x coordinates are close
    # TODO: create a test case for this approximation
    if np.isclose(b[0] - a[0], [0.], rtol=0.1).all():
        m = 0.000001
    else:
        m = (b[1] - a[1]) / (b[0] - a[0])

    # Use point a to solve the ordinate of the origin of the function
    # Because y = m*x + p ; y - m*x = p
    p: float = a[1] - m * a[0]
    if general_form:
        return lambda x, y: m * x + p - y
    return lambda x: m * x + p


def circle_gen_eq(center: np.array, r: float, general_form=True) -> Callable[[float, float], float]:
    """
    Returns the general form of the equation of a circle
    as (x - k)² + (y - h)² - r² = 0

    If general_form is set to false, we return the same equation but without the radius included
    as (x - k)² + (y - h)² = 0
    """
    if general_form:
        return lambda x, y: np.power(x - center[0], 2) + np.power(y - center[1], 2) - np.power(r, 2)
    return lambda x, y: np.power(x - center[0], 2) + np.power(y - center[1], 2)


def angle_towards(src: np.array, dst: np.array) -> float:
    """
    Returns the angle in radian from one point towards another one point
    """
    return np.arctan2(
        dst[1] - src[1],
        dst[0] - src[0],
    )


def compute_intersections(circle: Circle, line: tuple[np.array, np.array]) -> tuple[list[np.ndarray], bool]:
    """
    Using a circle and the source and two distinct points of a line, computes
    the number of crossing points between the circle and the line.

    To do this, it computes and uses the general equation of the given circle : (x - h)² + (y - k)² - r² = 0
    and the general form of the affine function : m*x + p - y = 0

    The order of given points in the 'line' argument is important :
        First is source point
        Second is destination point
    """

    # Important : set general_form to True to make it compliant with scipy.optimize.fsolve
    line_fc = traj_function(line[0], line[1], general_form=True)

    # Compute circle trajectory
    circle_fc = circle_gen_eq(circle.center, circle.r)

    # Use scipy.optimize.fsolve to get the intersection point(s)
    # This solves the equation circle_fc = line_fc
    # effectively computing the intersection points
    # The function might not find a proper root. We ask for the full output of the function
    # and only grab the returned roots and a special return value.

    roots = []
    equation = lambda xy: [line_fc(*xy), circle_fc(*xy)]
    intersect_point, _, retval, _ = scipy_fsolve(equation, np.array([-10, -10]), full_output=True)

    solution_found = retval == 1
    roots.append(intersect_point)

    # Artificially find the extra intersection
    artificial_intersect, valid = guess_extra_intersection(intersect_point, circle, circle_fc)
    if valid:
        roots.append(artificial_intersect)

    return roots, solution_found


def guess_extra_intersection(intersect: np.array, circle: Circle, circle_fc: Callable[[float, float], float]) -> tuple[np.array, bool]:
    """
    Finds a possible second intersection using a 180° symmetry
    If the obtained intersection is not in or on the circle, returns None
    """
    symmetrical_intersect = point_symmetry(intersect, circle.center)
    if np.isclose(circle_fc(*symmetrical_intersect), 0., rtol=0.01):
        return symmetrical_intersect, True
    return np.zeros(2), False


def point_symmetry(point: np.array, mirror: np.array):
    """
    Performs a 180° rotation of the given point, returning the mirror point
    """
    p_to_mir = mirror - point
    symmetrical = p_to_mir + mirror
    return symmetrical


def compute_waypoint(circle: Circle, line: tuple[np.array, np.array], rob_circles_avoid_list: list[Circle]) -> np.array:
    """
    Given a specific danger circle and two source and destination points,
    determines a waypoint to go to avoid said circle.

    The order of given points in the 'line' argument is important :
        First is source point
        Second is destination point
    TODO: change line param to vector
    TODO: refactor this mess, lots of rad->deg and deg->rad conversions everywhere

    This works by determining an orthogonal vector starting at the center of the danger circle
    towards the given line.
        Such is done by drawing a triangle, where the hypotenuse is the source point towards
        the circle center. We know that one angle will be of 90°, so we can calculate the last
        angle towards the line. This will be the angle of the vector

    Using the formula of this newfound vector, we can find the intersection of the vector with the line
    and compute the waypoint towards this intersection

    Note : every angle calculated here is in the range [0, 360]
           np.rad2deg() can output negative values, we cast the result mod 360
    TODO: thoroughly test this function
    """

    waypoint_vec_theta = triangle_normal_vec_theta(line, circle.center)

    # Using the circle's center point, and the found angle towards the line, we can compute
    # another point that is aligned to the vector
    r: float = 2.  # Arbitrary length, the value itself isn't important, but must be > 1
    aligned_pt: np.array = np.array(
        [circle.center[0] + r * np.cos(waypoint_vec_theta),  # | Polar to cartesian coordinates conversion
         circle.center[1] + r * np.sin(waypoint_vec_theta)]  # | x = r * cos(theta)  and  y = r * sin(theta)
    )

    # Get intersection between calculated vector and danger circle, which is the effective waypoint to attain
    intersects, _ = compute_intersections(circle=circle, line=(circle.center, aligned_pt))

    # Space waypoint away a little from circle
    possible_waypoints = space_away_from_circle(intersects, circle, rob_circles_avoid_list, src=line[0])

    # Waypoint to go to is the closest to the robot
    waypoint = closest_to_dst(possible_waypoints, line[1])

    return waypoint


def triangle_normal_vec_theta(line: tuple[np.array, np.array], point: np.array) -> float:
    """
    Using a source, destination, and an arbitrary point A,
    computes a vector normal to the line crossed by src and dst points,
    that starts at point A
    """

    # Calculate the angle DSC (dst -> src -> center of circle)
    SC_theta = np.rad2deg(angle_towards(line[0], point)) % 360
    SD_theta = np.rad2deg(angle_towards(line[0], line[1])) % 360
    DSC_angle = abs(SC_theta - SD_theta)  # Could fail if both angles are the same. What to do if this happens ?

    # We now need the angle of the vector that should go towards the line
    # Using the fact that sum of all angles of a triangle is 180, we know one given angle
    # and since we're looking for a right-angled triangle, last angle can be computed
    last_triangle_angle = 180 - DSC_angle - 90

    # Angle needs to be adapted to find the resulting vector
    CS_theta = np.rad2deg(angle_towards(point, line[0])) % 360  # Same as SC_theta + np.pi ?

    # Angle from circle center towards line is only the center->src angle plus
    # the computed angle of the triangle
    vec_normal_to_line_theta = np.deg2rad(CS_theta + last_triangle_angle)

    return vec_normal_to_line_theta


def space_away_from_circle(points: list[np.array], cir: Circle, circs_avoid: list[Circle], src: np.array) -> list[np.array]:
    """
    With a list of possible points to attain
    Move the given points a little further from the given circle's center, and by
    continually trying to avoid to put the waypoint in another danger circle
    Prints out a warning if it couldn't place a point out of a danger circle
    """
    result = []
    norm_vectors = [normalize_vec(p - cir.center) for p in points]

    for i in range(len(points)):
        p: np.array = points[i]
        unit_vec = norm_vectors[i]
        # TODO: put a point instead of None
        best_match = None

        # consider that best match is the point that uses the smallest k coefficient
        for k in np.arange(AVOID_DIST_FACTOR_MAX, 0, -0.1):
            point_away: np.array = p + unit_vec * k
            line = (src, point_away)
            # check if calculated point away is inside any danger circle that we should avoid
            # TODO: maybe this check can be replaced by using intersections with an affine function and circles ?
            inside_dgr_circles = np.array([
                # circle_gen_eq(c.center, c.r, general_form=False)(*point_away) < np.power(c.r, 2)
                compute_intersections(c, line)[1]
                for c in circs_avoid
            ])
            # keep current point away if previous condition is met
            if not inside_dgr_circles.any():
                best_match = point_away

        result.append(best_match)

    # # TODO: warn user if bot is gonna collide with someone (no best match found)
    # if None in result:
    #     print("Warning : space_away() couldn't return proper waypoints")

    return result


def normalize_vec(vec: np.array) -> np.array:
    """
    Normalizes the given vector
    """
    norm = np.sqrt(np.sum(vec ** 2))
    norm = 1 if norm == 0 else norm
    normalized_vec = vec / norm

    return normalized_vec


def closest_to_dst(points: list[np.array], origin: np.array) -> np.array:
    """
    Return the point that is the closest to the given destination point
    """
    if len(points) == 1:
        return points[0]

    # the None coalescing op is required to avoid subtracting to None
    # replaces None with absurdly far coordinates
    dist_to_robot = lambda xy: np.linalg.norm(origin - (xy if xy is not None else np.array([-100, -100])))
    wp_dists = list(map(dist_to_robot, points))
    index_min_dist = min(range(len(wp_dists)), key=wp_dists.__getitem__)
    return points[index_min_dist]

