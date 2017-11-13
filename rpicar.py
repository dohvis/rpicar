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
    LEFTMOSTLED,
    LEFTLESSLED,
    CENTERLED,
    RIGHTLESSLED,
    RIGHTMOSTLED
)
import time




def pwm_low():
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LEFT_PWM.ChangeDutyCycle(0)
    RIGHT_PWM.ChangeDutyCycle(0)
    GPIO.cleanup()


# chan_list = (11, 12)
# GPIO.output(chan_list, GPIO.LOW)  # all LOW
# GPIO.output(chan_list, (GPIO.HIGH, GPIO.LOW))  # first HIGH, second LOW


def _vertical_move(is_forward, speed, duration=None):
    _move_wheel(LEFT, is_forward, speed, duration)
    _move_wheel(RIGHT, is_forward, speed, duration)


def forward(speed, duration=None):
    _vertical_move('go', speed, duration)


def backward(speed, duration):
    _vertical_move('back', speed, duration)


def interface():
    """
    go 30 3;  sleep 1
    st 30 1 l
    pt 30 1 r
    :return:
    """
    cmds = {
        "g": forward, "b": backward,
        "st": swing_turn, "pt": point_turn,
    }
    input_cmd = input(">>> ")
    input_cmd.split(';')
    for cmd in input_cmd:
        argv = cmd.split()
        func = argv[0]

        args = argv[1:]

        try:
            cmds[func](*args)
        except KeyboardInterrupt:
            pwm_low()


def main():
    """
    1. 배터리 풀충
    2. 직진 보정 (속도 같게 할건지 다르게 줘야 하는지)
    3. 장애물이 다시 나타날 때 까지 회전 조금씩 하는거 구현
    4.
    """

    speed = 50
    forward_until_obstacle(speed)

    # st_speed = 70
    # swing_turn(st_speed, 2, 'r')
    #
    # pt_speed = 40
    # point_turn(pt_speed, 2, 'r')
    # get_distance()


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

    def line_trace(self):
        led = [LEFTMOSTLED, LEFTLESSLED, CENTERLED, RIGHTLESSLED, RIGHTMOSTLED]
        led_state = dict()

        for i in led:
            GPIO.setup(i, GPIO.IN)
            led_state[str(i)] = GPIO.input(i)
        
        return led_state


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
        speed = 50
        self.forward_until_obstacle(speed)
        self.point_turn(speed, 3, 'r')


if __name__ == '__main__':
    car = Car()
    car.run()

