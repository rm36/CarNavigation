# CarNavigation
Drives picar-4wd based on ultrasound and camera-detected objects

The entry point project.py is based on detect.py, the tflite example for raspberry pi under:
tflite > lite > examples > object_detection > raspberry_pi
As well as other dependencies such as object_detector and utils.

The policy for driving comes in with the driver.py file, which contains the main policy for driving.

To setup:
```
git clone https://github.com/sunfounder/picar-4wd
git clone https://github.com/tensorflow/examples
sh setup.sh
```
and then run:
```
python3 project.py
```
