import time
import cv2
from math import sin, cos, radians
from enum import Enum
import numpy as np

STOP_SIGN_LABEL = 'stop sign'

# Helper functions for the Driver class.
def has_stop_sign(detections):
    for detection in detections:
        class_name = detection.categories[0].label
        if class_name.lower() == STOP_SIGN_LABEL:
            return True
    return False


def display_stop_sign_detected(image):
    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 255, 0)  # green
    font_size = 1
    font_thickness = 1

    text_location = (left_margin, row_size/2)
    cv2.putText(image, 'Stop Sign', text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)


# Assuming the car is facing right.
def get_coordinates(angle, distance):
    x = distance * cos(radians(angle))
    y = distance * sin(radians(angle))
    return x, y


def transform_coordinates(global_angle, global_position, local_angle, distance):
    x = distance * cos(radians(global_angle + local_angle)) + global_position[0]
    y = distance * sin(radians(global_angle + local_angle)) + global_position[1]
    return x, y


def to_grid_space(global_measurement, cell_size, grid_origin):
    # Negative Y is down in the matrix
    result = (grid_origin[0] + int(global_measurement[0]/cell_size),
            grid_origin[1] + int(-global_measurement[1]/cell_size))
    if result[0] < 0 or result[1] < 0:
        return None 
    return result

class Movement(Enum):
    STOPPED = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


'''
Creates an online map, and navigates the picar to the target.
It takes into account the detected objects in the camera.
'''
class Driver:
    def __init__(self):
        # Camera detection
        self.stopSignDetectedTime = False
        
        # Moving scanner
        self.angleIndex = 0
        self.angles = [0] #[-30, -15, 0, 15, 30, 15, 0, -15]

        # Car position
        self.carAngle = 0
        self.carPosition = [0, 0]

        # Car movement
        self.movementStartTime = 0
        self.currentMovement = Movement.STOPPED
        self.currSpeed = 0

        # Grid
        self.grid = np.zeros((1000, 1000))
        self.gridLocation = (0, 0)
        self.gridTarget = (500, 500)

    ''' Main entry point to driving '''
    def drive_and_visualize(self, detections, image, car):
        self.update_location()

        if has_stop_sign(detections):
            display_stop_sign_detected(image)
            self.save_stop_sign_detected_time()

        self.break_if_needed(car)

        angle = self.get_angle_to_detect()
        distance = car.get_distance_at(angle)
        detectedAnything = True if distance > 0 else False
    
        xFromCar, yFromCar = get_coordinates(angle, distance)
        positionFromCar = (xFromCar, yFromCar)
        print('Distance from car:', distance, ', ultrasound angle:', self.get_ultrasound_angle(),
              'straight_d:', xFromCar)
        x, y = transform_coordinates(self.carAngle, self.carPosition, angle, distance)
        print('Global position:', (x, y))

        #target_direction, target_speed = navigation.get_target_direction()
        if xFromCar > 25:
            self.currSpeed = 10
            car.forward(self.currSpeed)
            self.currentMovement = Movement.FORWARD
        else:
            car.stop()
            self.currentMovement = Movement.STOPPED


    def update_location(self):
        timeElapsed = time.time() - self.movementStartTime
        ANGLE_SCALER = 1
        SPEED_SCALER = 1/1000000000
        if self.currentMovement == Movement.FORWARD:
            self.carPosition[0] += SPEED_SCALER * self.currSpeed * timeElapsed * cos(radians(self.carAngle))
            self.carPosition[1] += SPEED_SCALER * self.currSpeed * timeElapsed * sin(radians(self.carAngle))
        elif self.currentMovement == Movement.TURN_LEFT:
            self.carAngle -= ANGLE_SCALER * self.currSpeed * timeElapsed
        elif self.currentMovement == Movement.TURN_RIGHT:
            self.carAngle += ANGLE_SCALER * self.currSpeed * timeElapsed


    def save_stop_sign_detected_time(self):
        if not self.stopSignDetectedTime:
            self.stopSignDetectedTime = time.time()


    def break_if_needed(self, car):
        now = time.time()
        if self.stopSignDetectedTime + 3.0 >= now:
            car.stop()
            sleep(2)


    def get_angle_to_detect(self):
        self.angleIndex += 1
        return self.get_ultrasound_angle()

    
    def get_ultrasound_angle(self):
        return self.angles[self.angleIndex % len(self.angles)]


    def update_grid(self, measurements):
        for m in measurements:
            self.update_grid_pixels(m)
        return self.grid


    def update_grid_pixels(self, pixels, clearance = 0):
        eps = 1e-8
        for pixelX, pixelY in pixels:
            self.grid[pixelX, pixelY] = 1
            xStart = max(pixelX - clearance, 0)
            xEnd = min(pixelX + clearance + 1, self.grid.shape[0])
            yStart = max(pixelY - clearance, 0)
            yEnd = min(pixelY + clearance + 1, self.grid.shape[1])
            for x in range(xStart, xEnd):
                for y in range(yStart, yEnd):
                    # Inside a circle of size clearance
                    sqDist = (x-pixelX)*(x-pixelX)+(y-pixelY)*(y-pixelY)
                    if sqDist - eps <= clearance*clearance:
                        self.grid[x, y] = 1

        return self.grid
