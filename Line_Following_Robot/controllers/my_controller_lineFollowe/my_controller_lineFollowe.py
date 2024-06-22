import cv2
import numpy as np
from controller import Robot, Camera, Motor

robot = Robot()
TIME_STEP = 64


left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


camera = robot.getCamera("camera")
camera.enable(TIME_STEP)


kp = 0.1
ki = 0.01
kd = 0.05
integral = 0
previous_error = 0


colors_to_detect = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # Red, Green, Blue

def process_image(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    edges = cv2.Canny(mask, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x = x + w // 2
        return center_x
    else:
        return None

def pid_control(error):
    global integral, previous_error
    integral += error
    derivative = error - previous_error
    output = kp * error + ki * integral + kd * derivative
    previous_error = error
    return output

while robot.step(TIME_STEP) != -1:
    image = camera.getImageArray()
    image = np.array(image, dtype=np.uint8)
    image = np.reshape(image, (camera.getHeight(), camera.getWidth(), 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    line_position = process_image(image)
    
    if line_position is not None:
     
        error = line_position - camera.getWidth() // 2
       
        control_signal = pid_control(error)
        left_motor_speed = 5.0 - control_signal
        right_motor_speed = 5.0 + control_signal
    else:
        left_motor_speed = 0.0
        right_motor_speed = 0.0

   
    left_motor.setVelocity(left_motor_speed)
    right_motor.setVelocity(right_motor_speed)

 
    for x in range(0, camera.getWidth()):
        for y in range(0, camera.getHeight()):
            red = image[y, x, 2]
            green = image[y, x, 1]
            blue = image[y, x, 0]

            for color in colors_to_detect:
                if (red, green, blue) == color:
                    if color == (255, 0, 0):  # Red
                        left_motor.setVelocity(1.0)
                        right_motor.setVelocity(0.5)
                    elif color == (0, 255, 0):  # Green
                        left_motor.setVelocity(0.5)
                        right_motor.setVelocity(1.0)
                    elif color == (0, 0, 255):  # Blue
                        left_motor.setVelocity(1.0)
                        right_motor.setVelocity(1.0)
                    break
            else:
                left_motor.setVelocity(1.0)
                right_motor.setVelocity(1.0)

   
    cv2.imshow("Camera", image)
    cv2.waitKey(1)

cv2.destroyAllWindows()
del robot
