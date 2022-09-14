import RPi.GPIO as GPIO
from gpiozero import LineSensor, DistanceSensor
from time import sleep
from components import *

GPIO.cleanup()

motorsR = Motor(17, 22, 27)  # Right motors pins
motorsL = Motor(18, 23, 24)  # Left motors pins

right_sensor = LineSensor(20) # Right sensor pins
left_sensor = LineSensor(21)  # Left sensor pins

distance_sensor = DistanceSensor(6, 5)  # Distance sensor pins

servomotor = ServoMotor(13)  # Servomtor pin

camera = Camera()

rover = Rover(motorsR, motorsL, servomotor, right_sensor, left_sensor, distance_sensor)

while True:
    print("dist: " + str(distance_sensor.distance))
    if distance_sensor.distance < 0.3: # meters
        print("stop dist: " + str(distance_sensor.distance))
        rover.stop()
        sleep(0.1)
        rover.approach()
        rover.stop()
        sleep(0.2)
        text = camera.read()

        if text == "Right":
            rover.avoid("right", 38)

        elif text == "Left":
            rover.avoid("left", 38)

        elif text == "Push":
            rover.push()
            sleep(0.5)

    sleep(0.001)
    right_detect = int(right_sensor.value)
    left_detect  = int(left_sensor.value)

    if left_detect == 1 and right_detect == 1: # both sensors detect white
        rover.move("forward", 38)

    if left_detect == 1 and right_detect == 0: # right sensor detects black
        rover.move("right", 90)

    if left_detect == 0 and right_detect == 1: # left sensor detects white
        rover.move("left", 90)

    if left_detect == 0 and right_detect == 0: # both sensors detect black
        rover.stop()
        break
