import unittest
import numpy as np

from .basic_avoid_types import Circle
from .basic_avoid_utils import compute_intersections, is_behind_check


class BasicAvoidUtilsTest(unittest.TestCase):
    def test_compute_intersections(self):
        """
        Checks if basic_avoid_utils.compute_intersections() works properly
        You can visualize the functions easily by going on https://www.numworks.com/simulator/
        and use the 'Grapher' app on the website !
        """
        # -- Case 1 should fail, line doesn't cross circle --
        # Corresponding functions :
        #   Circle : (x-2)² + (y-3)² - 1 = 0
        #   Line   : 2x+3
        c1 = Circle(np.array([2, 3]), 1)
        l1_start = np.array([-3, -3])
        l1_end = np.array([3, 9])

        roots, sol_found = compute_intersections(circle=c1, line=(l1_start, l1_end))
        self.assertFalse(sol_found)

        # -- Case 2 should pass, line crosses circle --
        # Corresponding functions :
        #   Circle : (x-2)² + (y-3)² - 4 = 0
        #   Line   : 2x+3
        c2 = Circle(np.array([2, 3]), 4)
        l2_start = np.array([-3, -3])
        l2_end = np.array([3, 9])

        roots, sol_found = compute_intersections(circle=c2, line=(l2_start, l2_end))
        self.assertTrue(sol_found)

        # -- Case 3 should fail, intersection computed but the circle is behind the destination
        c3 = Circle(np.array([-2, 2]), 2)
        l3_start = np.array([0, 0])
        l3_end = np.array([2, -2])

        roots, sol_found = compute_intersections(circle=c3, line=(l3_start, l3_end), ignore_behind=True)
        self.assertFalse(sol_found)

    def test_is_behind_check(self):
        src = np.zeros(2)
        dst, p = np.array([0, 2]), np.array([2, 0])
        self.assertFalse(is_behind_check(src, dst, p))

        dst, p = np.array([2, 2]), np.array([3, 4])
        self.assertFalse(is_behind_check(src, dst, p))

        dst, p = np.array([0, 2]), np.array([-2, 2])
        self.assertFalse(is_behind_check(src, dst, p))

        dst, p = np.array([-2, 3]), np.array([2, 0])
        self.assertTrue(is_behind_check(src, dst, p))

        # Angle difference is slightly > 90°
        dst, p = np.array([2, 0]), np.array([-2, -0.05])
        self.assertTrue(is_behind_check(src, dst, p))


if __name__ == '__main__':
    unittest.main()
