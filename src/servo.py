from gpiozero import Servo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo = Servo(4, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

servo.mid()
sleep(10)
servo.max()
sleep(10)
servo.mid()
sleep(10)
servo.value = None;
