import time
import cv2
from math import sin, cos, radians, degrees, atan2
from enum import Enum
import numpy as np

STOP_SIGN_LABEL = 'person'
DISTANCE_PER_SECOND = 28 # cm / s
MOTOR_POWER = -10 # The power needed to go forward at DISTANCE_PER_SECOND
TIME_FOR_TURN_90 = 1.5 # Seconds of turn_right or turn_left

# Helper functions for the Driver class.
def has_stop_sign(detections):
    for detection in detections:
        class_name = detection.categories[0].label
        #print('detected:', class_name)
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
    result = (grid_origin[0] + global_measurement[0]/cell_size,
            grid_origin[1] + -global_measurement[1]/cell_size)
    return result

def get_distance_moved(timeElapsed, power):
    if power != MOTOR_POWER:
        print('Going at an uncalibrated speed! Please calibrate for speed', speed)
    return timeElapsed * DISTANCE_PER_SECOND

def get_angle_moved(timeElapsed, power):
    if power != MOTOR_POWER:
        print('Going at an uncalibrated speed! Please calibrate for speed', speed)
    return 90 * timeElapsed / TIME_FOR_TURN_90

class Movement(Enum):
    STOPPED = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


class CalibrationState(Enum):
    UNNECESSARY = 0
    PENDING = 1
    COMPLETED = 2


'''
Creates an online map, and navigates the picar to the target.
It takes into account the detected objects in the camera.
'''
class Driver:
    def __init__(self):
        # Camera detection
        self.stopSignDetectedTime = None

        # Target position
        self.targetGlobalPosition = (100.0, -30.0)  # cm

        # Grid
        self.grid = np.zeros((1000, 1000))
        self.gridOrigin = (500.0, 500.0)
        self.cellSize = 5 # cm per square side

        # from_grid_space()
        localTargetGridPosition = tuple(np.divide(self.targetGlobalPosition, self.cellSize))
        localTargetGridPosition = (localTargetGridPosition[0], -localTargetGridPosition[1])
        self.targetGridPosition = tuple(np.add(self.gridOrigin, localTargetGridPosition))

        # Moving ultrasound scanner. Loops through all angles
        self.angleIndex = 0
        self.angles = [0] #[-30, -15, 0, 15, 30, 15, 0, -15]

        # Car movement
        self.movementStartTime = 0
        self.currentMovement = Movement.STOPPED
        self.currPower = 0

        # Car position
        self.carAngle = 0
        self.carPosition = [0, 0] # Global coordinates in cm
        self.carGridPosition = self.gridOrigin

        # Calibration code. Edit the state and movement to calibrate below
        self.calibrationState = CalibrationState.UNNECESSARY
        self.calibrationMovement = Movement.FORWARD
        self.calibrationTime = TIME_FOR_TURN_90
        self.startMove = 0

        # Kill switch to avoid hitting people in case of a bug
        self.killSwitchTime = 7 # Seconds. value <= 0 means NO KILL SWITCH
        self.killSwitchStartTime = None

    ''' Main entry point to driving '''
    def drive_and_visualize(self, detections, image, car):

        # Kill switch logic
        if self.killSwitchTime > 0:
            if not self.killSwitchStartTime:
                self.killSwitchStartTime = time.time()
            else:
                if time.time() - self.killSwitchStartTime > self.killSwitchTime:
                    print('Kill switching off')
                    car.stop()
                    exit()

        if self.calibrationState != CalibrationState.UNNECESSARY:
            self.calibrate()
            return
        
        self.update_location()

        if has_stop_sign(detections):
            # Only display if not ssh'ing
            # display_stop_sign_detected(image)
            print('Stop sign detected! will break.')
            self.save_stop_sign_detected_time()

        self.break_if_needed(car)

        angle = self.get_angle_to_detect()

        # Do the detection with the ultrasound
        distance = car.get_distance_at(angle)
        detectedAnything = True if distance > 0 else False
        if detectedAnything:
            self.update_grid_with_ultrasound_detections(angle, distance)

        targetLocalAngle = self.get_car_target_direction()
        self.turn_angle(car, targetLocalAngle)

        if self.is_close_to_target():
            self.currPower = 0
            car.stop()
            self.currentMovement = Movement.STOPPED
            print('Got close enough!')
            exit()
        else:
            self.currPower = MOTOR_POWER
            car.forward(self.currPower)
            self.currentMovement = Movement.FORWARD
            self.movementStartTime = time.time()



    def calibrate(self):
        if self.calibrationState == CalibrationState.PENDING and self.currentMovement == Movement.STOPPED:
            self.currentMovement = self.calibrationMovement
            self.currPower = MOTOR_POWER
            if self.calibrationMovement == Movement.TURN_RIGHT:
                car.turn_right(self.currPower)
            elif self.calibrationMovement == Movement.FORWARD:
                car.forward(self.currPower)
            self.startMove = time.time()
        elif self.currentMovement == self.calibrationMovement:
            elapsedTime = time.time() - self.startMove
            if elapsedTime >= self.calibrationTime:
                self.currentMovement = Movement.STOPPED
                self.calibrationState = CalibrationState.COMPLETED
                car.stop()
                print('Elapsed time:', elapsedTime)
                exit()


    def update_location(self):
        timeElapsed = time.time() - self.movementStartTime
        if self.currentMovement == Movement.FORWARD:
            distance = get_distance_moved(timeElapsed, self.currPower)
            self.carPosition[0] += distance * cos(radians(self.carAngle))
            self.carPosition[1] += distance * sin(radians(self.carAngle))
            self.carGridPosition = to_grid_space(self.carPosition, self.cellSize, self.gridOrigin)
        elif self.currentMovement == Movement.TURN_LEFT:
            self.carAngle -= get_angle_moved(timeElapsed, self.currPower)
        elif self.currentMovement == Movement.TURN_RIGHT:
            self.carAngle += get_angle_moved(timeElapsed, self.currPower)


    def save_stop_sign_detected_time(self):
        if self.stopSignDetectedTime is None:
            self.stopSignDetectedTime = time.time()


    def break_if_needed(self, car):
        now = time.time()
        if self.stopSignDetectedTime and (self.stopSignDetectedTime + 3.0) >= now:
            print('breaking...')
            car.stop()
            time.sleep(1.5)
            self.stopSignDetectedTime = None


    def get_angle_to_detect(self):
        self.angleIndex += 1
        return self.get_ultrasound_angle()


    def update_grid_with_ultrasound_detections(self, angle, distance):
        xFromCar, yFromCar = get_coordinates(angle, distance)
        #print('Distance from car:', distance, ', ultrasound angle:', self.get_ultrasound_angle(),
        #      'straight_d:', xFromCar)
        x, y = transform_coordinates(self.carAngle, self.carPosition, angle, distance)
        #print('Global position:', (x, y))
        i, j = to_grid_space((x, y), self.cellSize, self.gridOrigin)
        self.update_grid_pixels([(i, j)], clearance = 3)

    
    def get_ultrasound_angle(self):
        return self.angles[self.angleIndex % len(self.angles)]


    def update_grid(self, measurements, clearance = 0):
        for m in measurements:
            self.update_grid_pixels(m, clearance)
        return self.grid


    def update_grid_pixels(self, pixels, clearance = 0):
        eps = 1e-8
        for pixelX, pixelY in pixels:
            self.grid[int(pixelX), int(pixelY)] = 1
            xStart = int(max(pixelX - clearance, 0))
            xEnd = int(min(pixelX + clearance + 1, self.grid.shape[0]))
            yStart = int(max(pixelY - clearance, 0))
            yEnd = int(min(pixelY + clearance + 1, self.grid.shape[1]))
            for x in range(xStart, xEnd):
                for y in range(yStart, yEnd):
                    # Inside a circle of size clearance
                    sqDist = (x-pixelX)*(x-pixelX)+(y-pixelY)*(y-pixelY)
                    if sqDist - eps <= clearance*clearance:
                        self.grid[x, y] = 1

        return self.grid


    def get_car_target_direction(self):
        # This should call the A* algorithm.
        deltaGridSpace = tuple(np.subtract(self.targetGridPosition, self.carGridPosition))
        print('target:',self.targetGridPosition,'car:', self.carGridPosition,'deltaGridSpace:', deltaGridSpace)

        # from_grid_space()
        deltaCm = tuple(np.multiply(deltaGridSpace, self.cellSize))
        deltaCm = (deltaCm[0], -deltaCm[1])

        angle = degrees(atan2(deltaCm[1], deltaCm[0]))
        print('Currently going at angle:', self.carAngle, 'should go at angle:', angle,
            'car position:',self.carPosition, 'delta in cm:', deltaCm)
        targetLocalAngle = angle - self.carAngle
        return targetLocalAngle


    def turn_angle(self, car, angle):
        absAngle = abs(angle)
        if absAngle < 4:
            return
        
        timeForTurn = absAngle * TIME_FOR_TURN_90 / 90
        print('turning ', angle, 'degrees, for seconds:', timeForTurn)
        
        if angle > 0:
            car.turn_left(MOTOR_POWER)
        else:
            car.turn_right(MOTOR_POWER)
        time.sleep(timeForTurn)
        print('done turning')

        # Update state
        self.carAngle += angle


    def is_close_to_target(self):
        deltaGridSpace = tuple(np.subtract(self.targetGridPosition, self.carGridPosition))
        deltaCm = tuple(np.multiply(deltaGridSpace, self.cellSize))
        if deltaCm[0] < 20 and deltaCm[1] < 20:
            return True
        return False