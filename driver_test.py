import unittest
import driver
import numpy as np

class FakeCategory:
    def __init__(self):
        self.label = ""

    def set_label(self, string):
        self.label = string


class FakeDetection:
    def __init__(self):
        self.categories = []

    def add_category(self, category):
        self.categories.append(category)

class TestDriver(unittest.TestCase):

    def test_hasnt_stop_sign(self):
        self.assertFalse(driver.has_stop_sign([]))

    def test_has_stop_sign(self):
        fakeCategory = FakeCategory()
        fakeCategory.set_label('person')
        fakeDetection = FakeDetection()
        fakeDetection.add_category(fakeCategory)
        detections = []
        detections.append(fakeDetection)
        self.assertTrue(driver.has_stop_sign(detections))

    def test_coordinates_at_45_deg(self):
        x, y = driver.get_coordinates(45, 10)
        self.assertAlmostEqual(x, y)

    def test_coordinates_at_30_deg(self):
        x, y = driver.get_coordinates(30, 10)
        self.assertAlmostEqual(y, 5)

    def test_transform_coordinates(self):
        global_position = [2, 2]
        global_angle = 45
        local_angle = 45
        distance = 7
        x, y = driver.transform_coordinates(global_angle, global_position, local_angle, distance)
        self.assertAlmostEqual(x, 2)
        self.assertAlmostEqual(y, 9)

    def test_driver_makes_big_empty_grid(self):
        d = driver.Driver()
        grid = d.update_grid([])
        self.assertTrue(grid.shape[0] > 100)
        self.assertTrue(grid.shape[1] > 100)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                self.assertEqual(grid[i,j], 0)

    def test_one_measurement(self):
        d = driver.Driver()
        grid = d.update_grid_pixels([(0,0), (0,1)], clearance=0)
        self.assertEqual(grid[0,0], 1)
        self.assertEqual(grid[0,1], 1)
        self.assertEqual(grid[0,2], 0)

    def test_one_big_measurement(self):
        d = driver.Driver()
        grid = d.update_grid_pixels([(0,0)], clearance=5)
        self.assertEqual(grid[0,0], 1)
        self.assertEqual(grid[0,5], 1)
        self.assertEqual(grid[0,6], 0)
        self.assertEqual(grid[5,0], 1)
        # Because of the shape of a circle, this should be out
        self.assertEqual(grid[5,4], 0)

    def test_big_measurements(self):
        d = driver.Driver()
        grid = d.update_grid_pixels([(0,0), (5,0), (10, 0)], clearance=5)
        for i in range(15):
            self.assertEqual(grid[i,0], 1)
        self.assertEqual(grid[16,0], 0)

    def test_to_grid_space(self):
        global_measurement = (12, -18)
        (gridX, gridY) = driver.to_grid_space(global_measurement, cell_size=5, grid_origin=(5,5))
        self.assertEqual(int(gridX), 5+2)
        self.assertEqual(int(gridY), 5+3)

if __name__ == '__main__':
    unittest.main()
