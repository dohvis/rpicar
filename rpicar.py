from RPi import GPIO
from config import (
    MotorLeft_A,
    MotorLeft_B,
    MotorRight_A,
    MotorRight_B,
    forward0,
    forward1,
    backward0,
    backward1,
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


LOOP = None


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

        self.left_flag = True
        self.right_flag = not self.left_flag
        self.forward_flag = True
        self.backward_flag = not self.left_flag
        # TODO: 이거 config로

        self.left_pwm = GPIO.PWM(MotorLeft_PWM, 100)
        self.right_pwm = GPIO.PWM(MotorRight_PWM, 100)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

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
                GPIO.output(MotorRight_A, GPIO.LOW)
                GPIO.output(MotorRight_B, GPIO.HIGH)
            self.right_pwm.ChangeDutyCycle(speed)

        if isinstance(duration, int):
            time.sleep(duration)
            self.stop()

    def stop(self):
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

    def forward_until_obstacle(self, speed):
        forward(speed)
        distance = self.get_distance()
        while distance > 3:
            distance = self.get_distance()

        self.stop()

    def get_distance(self):
        GPIO.setmode(GPIO.BOARD)

        # ultrasonic sensor setting
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        GPIO.output(TRIG, False)
        time.sleep(0.5)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(echo) == 0:
            pulse_start = time.time()

        while GPIO.input(echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        distance = round(distance, 2)
        return distance


    def get_led_states(self):
        led_states = [
            [LEFTMOST_LED, True], # PIN_NUM, IS_ONLINE
            [LEFTLESS_LED, True],
            [CENTER_LED, True],
            [RIGHTLESS_LED, True],
            [RIGHTMOST_LED, True],
        ]

        for i, v in enumerate(led_states):
            pin_num, is_line = v
            GPIO.setup(pin_num, GPIO.IN)
            led_states[i][1] = (GPIO.input(pin_num) == 1)
        return led_states

    def determine_direction(self):
        states = self.get_led_states()
        online_sensors = [(i-2) for i, state in enumerate(states) if state[1] == True]
        print(sum(online_sensors))



    def line_trace(self):
        l_speed = 50
        r_speed = 50
        while True:
            self.determine_direction()



    def swing_turn(self, speed, duration, direction):
        if direction == 'l':
            # left
            self.move_wheel(self.right_flag, self.forward_flag, speed, duration)
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

    def run(self):
        self.line_trace()


if __name__ == '__main__':
    car = Car()
    try:
        car.run()
    except KeyboardInterrupt:
        GPIO.cleanup()
    GPIO.cleanup()
