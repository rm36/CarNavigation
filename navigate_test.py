import unittest
from Navigate import a_star_search
import numpy as np

class TestAStar(unittest.TestCase):
    def test_simplest_2d_grid(self):
        grid = np.array([[0, 0],
                         [0, 0]])
        path = a_star_search(grid, startPosition=(0,0), endPosition=(1,1))
        self.assertEqual(path, [(0,0), (1,1)])

    def test_line(self):
        grid = np.array([[0, 0, 0, 0]])
        path = a_star_search(grid, startPosition=(0,0), endPosition=(0,3))
        self.assertEqual(path, [(0,0), (0,1), (0,2), (0,3)])

    def test_obstacle(self):
        grid = np.array([[0, 1, 1, 0],
                         [0, 0, 0, 0],
                         [0, 1, 1, 0]])
        path = a_star_search(grid, startPosition=(0,0), endPosition=(2,3))
        self.assertEqual(path, [(0,0), (1,1), (1,2), (2,3)])

    def test_backwards_path(self):
        grid = np.array([[0, 1, 1, 0],
                         [0, 0, 0, 0],
                         [0, 1, 1, 0]])
        path = a_star_search(grid, startPosition=(2,3), endPosition=(0,0))
        self.assertEqual(path, list(reversed([(0,0), (1,1), (1,2), (2,3)])))

    def test_maze(self):
        grid = np.array([[0, 1, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0],
                         [1, 1, 1, 1, 0, 0],
                         [0, 0, 0, 0, 0, 1],
                         [0, 1, 1, 1, 1, 1],
                         [0, 0, 0, 0, 0, 0]])
        path = a_star_search(grid, startPosition=(0,0), endPosition=(5,5))
        self.assertEqual(len(path), 15)
        self.assertTrue((3, 3) in path)

if __name__ == '__main__':
    unittest.main()
