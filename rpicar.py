from RPi import GPIO
from config import (
    MotorLeft_A,
    MotorLeft_B,
    MotorRight_A,
    MotorRight_B,
    MotorLeft_PWM,
    MotorRight_PWM,
    TRIG,
    ECHO,
    LEFTMOST_LED,
    LEFTLESS_LED,
    CENTER_LED,
    RIGHTLESS_LED,
    RIGHTMOST_LED
)
import time
from ctypes import c_double
from multiprocessing import Process, Value, Lock

LOCK = Lock()
DISTANCE = Value(c_double, 0.0)


def set_distance():
    try:
        import os
        GPIO.setmode(GPIO.BOARD)

        # ultrasonic sensor setting
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        while True:
            GPIO.output(TRIG, False)
            time.sleep(0.3)
            GPIO.output(TRIG, True)
            time.sleep(0.00001)
            GPIO.output(TRIG, False)

            while GPIO.input(ECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(ECHO) == 1:
                pulse_end = time.time()

            # from random import random
            # pulse_duration = random() - random()
            pulse_duration = pulse_end - pulse_start

            distance = pulse_duration * 17000
            distance = round(distance, 2)
            with LOCK:
                DISTANCE.value = distance
    except KeyboardInterrupt:
        GPIO.cleanup()


class Car:
    def __init__(self):
        # set GPIO warnings as flase
        GPIO.setwarnings(False)

        # set up GPIO mode as BOARD
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(MotorLeft_A, GPIO.OUT)
        GPIO.setup(MotorLeft_B, GPIO.OUT)
        GPIO.setup(MotorLeft_PWM, GPIO.OUT)

        """
        모터와 라즈베리파이의 연결이 성립되면, 라즈베리파이의 GPI 핀들(MotorLeft_A, MotorLeft_B, and MotorLeft_PWM) 은
        핀의 역할이 출력(output)인지, 혹은 입력(input)인지 여부를 명확하게 선언해야합니다.
        """

        GPIO.setup(MotorRight_A, GPIO.OUT)
        GPIO.setup(MotorRight_B, GPIO.OUT)
        GPIO.setup(MotorRight_PWM, GPIO.OUT)

        GPIO.setup(CENTER_LED, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # led setup
        GPIO.setup(LEFTLESS_LED, GPIO.IN)
        GPIO.setup(LEFTMOST_LED, GPIO.IN)
        GPIO.setup(CENTER_LED, GPIO.IN)
        GPIO.setup(RIGHTLESS_LED, GPIO.IN)
        GPIO.setup(RIGHTMOST_LED, GPIO.IN)

        self.left_flag = True
        self.right_flag = False
        self.forward_flag = True
        self.backward_flag = False
        # TODO: 이거 config로

        self.left_pwm = GPIO.PWM(MotorLeft_PWM, 100)
        self.right_pwm = GPIO.PWM(MotorRight_PWM, 100)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

        self.lock = Lock()

    def move_wheel(self, is_left, is_forward, speed, duration=None):
        if is_left:
            if is_forward:
                GPIO.output(MotorLeft_A, GPIO.HIGH)
                GPIO.output(MotorLeft_B, GPIO.LOW)
            else:
                GPIO.output(MotorLeft_A, GPIO.LOW)
                GPIO.output(MotorLeft_B, GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(speed)
        else:
            if is_forward:
                GPIO.output(MotorRight_A, GPIO.HIGH)
                GPIO.output(MotorRight_B, GPIO.LOW)
            else:
                GPIO.output(MotorRight_A, GPIO.HIGH)
                GPIO.output(MotorRight_B, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(speed)

        if isinstance(duration, int):
            time.sleep(duration)
            self.stop()

    def go_forward(self, left_speed, right_speed, line_state, duration):
        # left
        GPIO.output(MotorLeft_A, GPIO.LOW)
        GPIO.output(MotorLeft_B, GPIO.HIGH)
        self.left_pwm.ChangeDutyCycle(left_speed)
        # right
        GPIO.output(MotorRight_A, GPIO.HIGH)
        GPIO.output(MotorRight_B, GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(right_speed)

        if duration is not None:
            time.sleep(duration)
        if line_state == [1, 1, 1, 1, 1]:
            self.stop()

    def stop(self):
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

    def forward_until_obstacle(self, speed):
        # go_forward(speed, speed, )
        distance = self.distance.value
        while distance > 3:
            distance = self.distance.value
        self.stop()

    def get_led_states(self):
        led_list = [LEFTMOST_LED, LEFTLESS_LED, CENTER_LED, RIGHTLESS_LED, RIGHTMOST_LED]
        led_state = list()

        for led in led_list:
            led_state.append(str(GPIO.input(led)))

        return led_state

    def swing_turn(self, speed, duration, direction):
        if direction == 'l':
            # left
            self.move_wheel(self.right_flag, not self.forward_flag, speed, duration)
        else:
            self.move_wheel(self.left_flag, self.forward_flag, speed, duration)

    def point_turn(self, speed, duration, direction):
        if direction == 'l':
            # left
            self.move_wheel(self.right_flag, self.forward_flag, speed, duration)
            self.move_wheel(self.left_flag, self.backward_flag, speed, duration)
        else:
            self.move_wheel(self.left_flag, self.forward_flag, speed, duration)
            self.move_wheel(self.right_flag, self.backward_flag, speed, duration)

    def avoid_obstacle(self):
        print("Right")
        _speed = 25
        time.sleep(1)

        self.go_forward(45, 0, ['0', '0', '0', '0', '0', ], 0.5)
        self.stop()
        time.sleep(1)

        print("Curve")
        self.go_forward(30, 30, ['0', '0', '0', '0', '0', ], 0.25)
        self.stop()

        while True:
            self.go_forward(15, 50, ['0', '0', '0', '0', '0'], 0.01)
            leds = self.get_led_states()
            if '0' in leds:
                self.go_forward(50, 20, ['0', '0', '0', '0', '0', ], 0.1)
                self.stop()
                print('-' * 20, leds)
                break

    def run(self):
        self.stop()
        try:
            self.go_forward(25, 25, ['0', '0', '0', '0', '0'], 1)
            while True:
                line_check = self.get_led_states()
                if 5.0 < DISTANCE.value < 25.0:
                    print("STOP!", DISTANCE.value)
                    self.stop()
                    time.sleep(1)
                    self.avoid_obstacle()
                else:
                    print(line_check)
                    pass

                if line_check == ['0', '1', '1', '1', '1']:
                    self.go_forward(5, 75, line_check, 0.01)
                elif line_check == ['1', '0', '1', '1', '1']:
                    self.go_forward(25, 35, line_check, 0.01)
                elif line_check == ['1', '1', '0', '1', '1']:
                    self.go_forward(35, 35, line_check, 0.01)
                elif line_check == ['1', '1', '1', '0', '1']:
                    self.go_forward(35, 25, line_check, 0.01)
                elif line_check == ['1', '1', '1', '1', '0']:
                    self.go_forward(75, 10, line_check, 0.01)
                elif line_check == ['0', '0', '1', '1', '1']:
                    self.go_forward(20, 45, line_check, 0.01)
                elif line_check == ['1', '0', '0', '1', '1']:
                    self.go_forward(30, 20, line_check, 0.005)
                elif line_check == ['1', '1', '0', '0', '1']:
                    self.go_forward(25, 45, line_check, 0.01)
                elif line_check == ['1', '1', '1', '0', '0']:
                    self.go_forward(40, 20, line_check, 0.01)
                elif line_check == ['1', '1', '0', '0', '0']:
                    self.go_forward(25, 40, line_check, 0.01)
                elif line_check == ['1', '0', '0', '0', '1']:
                    self.go_forward(30, 30, line_check, 0.01)
                elif line_check == ['0', '0', '0', '1', '1']:
                    self.go_forward(30, 15, line_check, 0.01)
                elif line_check == ['1', '1', '1', '1', '1']:
                    self.go_forward(35, 35, line_check, 0.01)
                elif line_check == ['0', '0', '0', '0', '1']:
                    self.go_forward(45, 30, line_check, 0.01)
                elif line_check == ['1', '0', '0', '0', '0']:
                    self.go_forward(20, 55, line_check, 0.01)
                else:
                    self.stop()
        except KeyboardInterrupt:
            GPIO.cleanup()


def main():
    car = Car()
    car.run()


if __name__ == '__main__':
    try:
        distance_proc = Process(target=set_distance)
        main_proc = Process(target=main)
        distance_proc.start()
        main_proc.start()
        distance_proc.join()
        main_proc.join()
    except KeyboardInterrupt:
        GPIO.cleanup()
