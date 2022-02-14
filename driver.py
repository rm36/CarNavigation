import time
import cv2
from math import sin, cos, radians, degrees, atan2
from enum import Enum
import numpy as np
from navigate import a_star_search

STOP_SIGN_LABEL = 'teddy bear' # What I can use at home as a stop sign :)
DISTANCE_PER_SECOND = 28 # cm / s
MOTOR_POWER = -10 # The power needed to go forward at DISTANCE_PER_SECOND
TIME_FOR_TURN_90 = 1.4 # Seconds of turn_right or turn_left

# Helper functions for the Driver class.
def has_stop_sign(detections):
    for detection in detections:
        class_name = detection.categories[0].label
        print('detected:', class_name)
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
    x = distance * (cos(radians(angle)))
    y = distance * (sin(radians(angle)))
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

def to_metric_space(gridCoordinates, cellSize):
    metricCoordinates = tuple(np.multiply(gridCoordinates, cellSize))
    metricCoordinates = (metricCoordinates[0], -metricCoordinates[1])
    return metricCoordinates

def get_distance_moved(timeElapsed, power):
    if power == 0:
        return 0

    if power != MOTOR_POWER:
        print('Going at an uncalibrated speed! Please calibrate for speed', speed)
    return timeElapsed * DISTANCE_PER_SECOND

def get_angle_moved(timeElapsed, power):
    if power != MOTOR_POWER:
        print('Going at an uncalibrated speed! Please calibrate for speed', speed)
    return 90 * timeElapsed / TIME_FOR_TURN_90

def find_min_ranges(grid, mustIncludeCells):
    minX, minY = 10000, 10000
    maxX,maxY = 0, 0
    for (i, j) in mustIncludeCells:
        minX = i if i < minX else minX
        maxX = i if i > maxX else maxX
        minY = j if j < minY else minY
        maxY = j if j > maxY else maxY

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] > 0:
                minX = i if i < minX else minX
                maxX = i if i > maxX else maxX
                minY = j if j < minY else minY
                maxY = j if j > maxY else maxY
    return minX, maxX, minY, maxY

def print_grid(grid, carPosition, targetPosition):
    start = to_index(carPosition)
    end = to_index(targetPosition)
    minX, maxX, minY, maxY = find_min_ranges(grid, mustIncludeCells = [start, end])
    print('----GRID----', minX, minY)
    subGrid = np.copy(grid[minX:maxX+1, minY:maxY+1])
    localStart = (start[0] - minX, start[1] - minY)
    localEnd = (end[0] - minX, end[1] - minY)
    subGrid[localStart] = 2
    subGrid[localEnd] = 3
    print(np.flip(subGrid, axis=0))

def print_path(grid, carPosition, targetPosition, path):
    start = to_index(carPosition)
    end = to_index(targetPosition)
    mustIncludeCells = [start, end]
    mustIncludeCells.extend(path)
    minX, maxX, minY, maxY = find_min_ranges(grid, mustIncludeCells)
    print('----GRID PATH----', minX, minY)
    subGrid = np.copy(grid[minX:maxX+1, minY:maxY+1])
    localStart = (start[0] - minX, start[1] - minY)
    localEnd = (end[0] - minX, end[1] - minY)
    subGrid[localStart] = 2
    subGrid[localEnd] = 3
    for cell in path:
        if cell != start and cell != end:
            localCell = (cell[0] - minX, cell[1] - minY)
            subGrid[localCell] = 5
    print(np.flip(subGrid, axis=0))

def to_index(coor):
    return (int(coor[0]), int(coor[1]))

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
        self.targetGlobalPosition = (200.0, 280.0)  # cm

        # Grid
        size = (500,500)
        self.grid = np.zeros(size)
        self.gridOrigin = (0,450)#(tuple(np.divide(size, 2)))
        self.cellSize = 5 # cm per square side
        self.targetGridPosition = to_grid_space(self.targetGlobalPosition, self.cellSize, self.gridOrigin)

        # Moving ultrasound scanner. Loops through all angles
        self.angleIndex = 0
        self.angles = [50, 30, 15, 0]
        self.lastScannedPoint = None

        # Car movement
        self.movementStartTime = 0
        self.currentMovement = Movement.STOPPED
        self.currPower = 0
        self.startTime = None

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
        self.killSwitchTime = -1 # Seconds. value <= 0 means NO KILL SWITCH
        self.killSwitchStartTime = None

        # Printing
        self.lastPrintedStatus = None

        np.set_printoptions(edgeitems=50, linewidth=100000, formatter=dict(float=lambda x: "%.3g" % x))

    ''' Main entry point to driving '''
    def drive_and_visualize(self, detections, image, car):
        self.kill_switch_if_enabled(car)

        if self.calibrationState != CalibrationState.UNNECESSARY:
            self.calibrate()
            return

        self.update_location()

        if has_stop_sign(detections):
            # Only display if not ssh'ing
            # display_stop_sign_detected(image)
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=================                                     ===================')
            print('=================   PENGUIN  DETECTED       BREAKING  ===================')
            print('=================                                     ===================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
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

        self.drive_forward_as_needed(car)


    def kill_switch_if_enabled(self, car):
        if self.killSwitchTime > 0:
            if not self.killSwitchStartTime:
                self.killSwitchStartTime = time.time()
            else:
                if time.time() - self.killSwitchStartTime > self.killSwitchTime:
                    print('Kill switching off')
                    car.stop()
                    print_grid(self.grid, self.carGridPosition, self.targetGridPosition)
                    exit()

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
        if self.stopSignDetectedTime is None and self.stopSignDetectedTime != -1:
            self.stopSignDetectedTime = time.time()


    def break_if_needed(self, car):
        # -1 means don't ever break again
        if self.stopSignDetectedTime and self.stopSignDetectedTime == -1:
            return

        now = time.time()
        if self.stopSignDetectedTime and (self.stopSignDetectedTime + 5.0) >= now:
            print('Saw a stop sign! Breaking...')
            car.stop()
            time.sleep(2.0)
            self.stopSignDetectedTime = -1


    def get_angle_to_detect(self):
        self.angleIndex += 1
        return self.get_ultrasound_angle()


    def update_grid_with_ultrasound_detections(self, angle, distance):
        #print('Distance from car:', distance, ', ultrasound angle:', self.get_ultrasound_angle())
        x, y = transform_coordinates(self.carAngle, self.carPosition, angle, distance)
        i, j = to_grid_space((x, y), self.cellSize, self.gridOrigin)
        #print('Grid ultrasound:', (i,j))

        if self.lastScannedPoint is not None:
            (lastX, lastY) = self.lastScannedPoint
            diagDistance = max(abs(i - lastX), abs(j - lastY))
            if diagDistance * self.cellSize < 60: # cm is the smallest hole to navigate through
                # Fill in the mid point to avoid gaps
                midpoint = ((lastX + i) / 2, (lastY + j) / 2)
                self.update_grid_pixel(midpoint, clearance = 5)

        # Clear the empty space that should be empty
        midpointFromCar = ((i + self.carGridPosition[0])/2, (j + self.carGridPosition[1])/2)
        self.update_grid_pixel(midpointFromCar, clearance = 3, value = 0)
        midpointFromCar = ((i + self.carGridPosition[0]*3)/4, (j + self.carGridPosition[1]*3)/4)
        self.update_grid_pixel(midpointFromCar, clearance = 3, value = 0)
        self.update_grid_pixel(self.carGridPosition, clearance = 3, value = 0)

        # Fill the space detected
        self.update_grid_pixel((i, j), clearance = 5)
        self.lastScannedPoint = (i, j)

    
    def get_ultrasound_angle(self):
        return self.angles[self.angleIndex % len(self.angles)]


    def set_grid_pixel_value(self, coor, value):
        if coor[0] < 0 or coor[1] < 0 or coor[0] >= self.grid.shape[0] or coor[1] >= self.grid.shape[1]:
            return
        self.grid[coor] = value


    def update_grid_pixel(self, pixel, clearance = 0, value = 1):
        eps = 1e-8
        (pixelX, pixelY) = (int(pixel[0]), int(pixel[1]))
        self.set_grid_pixel_value((pixelX, pixelY), value)
        xStart = max(pixelX - clearance, 0)
        xEnd = min(pixelX + clearance + 1, self.grid.shape[0])
        yStart = max(pixelY - clearance, 0)
        yEnd = min(pixelY + clearance + 1, self.grid.shape[1])
        for x in range(xStart, xEnd):
            for y in range(yStart, yEnd):
                # Inside a circle of size clearance
                sqDist = (x-pixelX)*(x-pixelX)+(y-pixelY)*(y-pixelY)
                if sqDist - eps <= clearance*clearance:
                    self.set_grid_pixel_value((x, y), value)
        return self.grid


    def get_car_target_direction(self):
        idealPath = a_star_search(self.grid, to_index(self.carGridPosition), to_index(self.targetGridPosition))

        if not idealPath:
            print('Something went wrong, no path to end')
            print_grid(self.grid, self.carGridPosition, self.targetGridPosition)
            size = (500,500)
            self.grid = np.zeros(size)
            idealPath = []

        targetCell = self.targetGridPosition
        if len(idealPath) < 8:
            print('Short path!!!', repr(idealPath))
        else:
            targetCell = idealPath[7]

        deltaGridSpace = tuple(np.subtract(targetCell, self.carGridPosition))
        deltaCm = to_metric_space(deltaGridSpace, self.cellSize)
        angle = degrees(atan2(deltaCm[1], deltaCm[0]))

        if self.lastPrintedStatus is None or (time.time() - self.lastPrintedStatus) > 12.0:
            self.lastPrintedStatus = time.time()
            print('----CAR STATUS----')
            print('Target:', targetCell, 'Car:', self.carGridPosition,
                  'DeltaGridSpace:', deltaGridSpace)
            print('Currently going at angle:', self.carAngle, 'Should go at angle:', angle,
                  'Car position:', self.carPosition, 'Delta in cm:', deltaCm)

            print_path(self.grid, self.carGridPosition, self.targetGridPosition, idealPath)

        targetLocalAngle = angle - self.carAngle
        return targetLocalAngle


    def turn_angle(self, car, angle):
        absAngle = abs(angle)
        if absAngle < 4:
            return
        
        timeForTurn = absAngle * TIME_FOR_TURN_90 / 90
        #print('turning ', angle, 'degrees, for seconds:', timeForTurn)
        
        if angle > 0:
            car.turn_left(MOTOR_POWER)
        else:
            car.turn_right(MOTOR_POWER)
        time.sleep(timeForTurn)
        #print('done turning')

        # Update state
        self.carAngle += angle


    def drive_forward_as_needed(self, car):
        if self.is_close_to_target():
            self.currPower = 0
            car.stop()
            self.currentMovement = Movement.STOPPED
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=================                                       =================')
            print('=================   ARRIVED AT DESTINATION... STOPPING  =================')
            print('=================                                       =================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            print('=========================================================================')
            #print_grid(self.grid, self.carGridPosition, self.targetGridPosition)
            exit()
        elif self.just_started():
            self.currPower = 0
            car.stop()
            self.currentMovement = Movement.STOPPED
            print_grid(self.grid, self.carGridPosition, self.targetGridPosition)
        else:
            self.currPower = MOTOR_POWER
            car.forward(self.currPower)
            self.currentMovement = Movement.FORWARD
            self.movementStartTime = time.time()


    def is_close_to_target(self):
        deltaGridSpace = tuple(np.subtract(self.targetGridPosition, self.carGridPosition))
        deltaCm = tuple(np.multiply(deltaGridSpace, self.cellSize))
        if deltaCm[0] < 20 and deltaCm[1] < 20:
            return True
        return False

    def just_started(self):
        if self.startTime is None:
            self.startTime = time.time()
        return (time.time() - self.startTime) < 5