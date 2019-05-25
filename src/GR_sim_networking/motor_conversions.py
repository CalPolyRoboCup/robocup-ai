import math

motor_angles = [math.pi/3, 3*math.pi/4, 5*math.pi/4, 5*math.pi/3]

def get_motor_speeds(tangent, norm, angle):
    motor_speeds = [(1.0 / WHEEL_RADIUS) * ((ROBOT_RADIUS * angle) - (tangent * math.sin(motor_angles[i])) + (norm * math.cos(motor_angles[i]))) for i in range(4)]
    return motor_speeds