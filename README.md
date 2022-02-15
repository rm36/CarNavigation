# CarNavigation
Drives picar-4wd based on ultrasound and camera-detected objects

[Link to demo video](https://drive.google.com/file/d/1QYt0iIC6UFFmXKSnCSA72veqJqOxWv0N/view)

[Link to explanation video](https://drive.google.com/file/d/1Q3Fz3PPjt0I4e8oi2Ovz8yiAZJKmhkP2/view)

[Link to Report](Lab1Report.pdf)


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

## Code description

- The entry point [project.py](project.py) is based on detect.py, the tflite example for raspberry pi under:
tflite > lite > examples > object_detection > raspberry_pi
As well as other dependencies such as object_detector and utils.

- The policy for driving comes in with the [driver.py](driver.py) file, which contains the main policy for driving.

- The A* navigation code is in [navigate.py](navigate.py). See tests for more details.
