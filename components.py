import RPi.GPIO as GPIO
from gpiozero import LineSensor, DistanceSensor
from time import sleep
from PIL import Image
from picamera import PiCamera
import pytesseract

class Motor:
    def __init__(self, pwm_pin, dir1, dir2):
        self.pwm_pin = pwm_pin
        self.dir1 = dir1
        self.dir2 = dir2
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dir1,GPIO.OUT)
        GPIO.setup(dir2,GPIO.OUT)
        GPIO.setup(pwm_pin,GPIO.OUT)
        GPIO.output(dir1,GPIO.LOW)
        GPIO.output(dir2,GPIO.LOW)
        
        self.pwm = GPIO.PWM(pwm_pin, 1000)
    
    def actuate(self, direction, duty_cycle):
        if direction == "cw":
            GPIO.output(self.dir1, GPIO.HIGH)
            GPIO.output(self.dir2, GPIO.LOW)

        elif direction == "ccw":
            GPIO.output(self.dir1, GPIO.LOW)
            GPIO.output(self.dir2, GPIO.HIGH)

        else:
            print("direction must be cw or ccw")
            return

        self.pwm.start(duty_cycle)
    
    def stop(self):
        GPIO.output(self.dir1, GPIO.LOW)
        GPIO.output(self.dir2, GPIO.LOW)

class Rover:
    def __init__(self, motorR, motorL, servomotor, right_sensor, left_sensor, distance_sensor):
        self.motorR = motorR
        self.motorL = motorL
        self.servomotor = servomotor
        self.right_sensor = right_sensor
        self.left_sensor = left_sensor
        self.distance_sensor = distance_sensor
    
    def move(self, direction, duty_cycle = 50):
        
        if direction == "forward":
            self.motorR.actuate("cw", duty_cycle)
            self.motorL.actuate("cw", duty_cycle)
            #print("f")        
        
        elif direction == "backward":
            self.motorR.actuate("ccw", duty_cycle)
            self.motorL.actuate("ccw", duty_cycle)
        
        elif direction == "right":
            self.motorR.actuate("ccw", duty_cycle)
            self.motorL.actuate("cw", duty_cycle)
            #print("r")        

        elif direction == "left":
            self.motorR.actuate("cw", duty_cycle)
            self.motorL.actuate("ccw", duty_cycle)
            #print("l")        

        else:
            print("direction must be forward, backward, left, right")
        
    def stop(self):
        self.motorR.stop()
        self.motorL.stop()
    
    def approach(self):
        while self.distance_sensor.distance > 0.13:
            self.move("forward", 45)
            sleep(0.1)
            self.stop()
            sleep(0.05)
            print("app dist: " + str(self.distance_sensor.distance))

    def avoid(self, direction, duty_cycle = 50):
        self.move("backward", 55)
        sleep(0.7)
        self.stop()
        if direction == "right":
            self.move("right", 80)
            sleep(0.6)
            self.move("forward", duty_cycle)
            sleep(2.7)
            self.move("left", 80)
            sleep(1.1)
            print("abl while")
            self.stop()
            sleep(0.2)
            while True:
               self.move("forward", 38)
               r = int(self.right_sensor.value)
               print("right sens: " + str(r))
               if r==0:
                   self.move("right",100)
                   sleep(0.3)
                   self.stop()
                   sleep(0.2)
                   break

        elif direction == "left":
            self.move("left", 80)
            sleep(0.6)
            self.move("forward", duty_cycle)
            sleep(2.3)
            self.move("right", 100)
            sleep(0.7)
            print("abl while")
            self.stop()
            sleep(0.2)
            while True:
               self.move("forward", duty_cycle)
               l = int(self.left_sensor.value)
               print("left sens: " + str(l))
               if l==0:
                   self.move("left",100)
                   sleep(0.3)
                   self.stop()
                   sleep(0.2)
                   break

        else:
            print("direction should be right or left")

    def push(self):
        self.servomotor.servo_pwm.start(2.5)
        self.servomotor.servo_pwm.ChangeDutyCycle(11)
        sleep(2)
        self.servomotor.servo_pwm.ChangeDutyCycle(5)
        sleep(2)
        self.servomotor.servo_pwm.ChangeDutyCycle(11)
        sleep(2)
        self.servomotor.servo_pwm.stop()

class ServoMotor:
    def __init__(self, servo_pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(servo_pin, 50)

class Camera:
    def __init__(self):
        self.picamera = PiCamera()
    
    def read(self):
        print("camera reading...")
        self.picamera.start_preview()
        sleep(0.5)
        self.picamera.capture('/home/pi/Desktop/MCE/sign.png')
        self.picamera.stop_preview()
        img =Image.open('sign.png')
        text = pytesseract.image_to_string(img, config='')
        print('Camera read: ' + text)
        return text.split()[0]
    

