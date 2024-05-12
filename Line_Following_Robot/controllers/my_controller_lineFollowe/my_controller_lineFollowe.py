from controller import Robot, Camera, Motor


TIME_STEP = 64


robot = Robot()


camera = robot.getCamera("camera")
camera.enable(TIME_STEP)


left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


while robot.step(TIME_STEP) != -1:
    image = camera.getImage()

    
    colors_to_detect = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

    
    for x in range(0, camera.getWidth()):
        for y in range(0, camera.getHeight()):
           
            red = image[y * camera.getWidth() * 4 + x * 4]
            green = image[y * camera.getWidth() * 4 + x * 4 + 1]
            blue = image[y * camera.getWidth() * 4 + x * 4 + 2]

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


del robot
