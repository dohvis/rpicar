from RPi import GPIO

forward0 = True
forward1 = False

backward0 = not forward0
backward1 = not forward1

# =======================================================================
# declare the pins of 12, 11, 35 in the Rapberry Pi
# as the left motor control pins in order to control left motor
# left motor needs three pins to be controlled
# =======================================================================
MotorLeft_A = 12
MotorLeft_B = 11
MotorLeft_PWM = 35

# =======================================================================
# declare the pins of 15, 13, 37 in the Rapberry Pi
# as the right motor control pins in order to control right motor
# right motor needs three pins to be controlled
# =======================================================================
MotorRight_A = 15
MotorRight_B = 13
MotorRight_PWM = 37

# =======================================================================
# create left pwm object to control the speed of left motor
# =======================================================================

# =======================================================================
# create right pwm object to control the speed of right motor
# =======================================================================

TRIG = 33
ECHO = 31
