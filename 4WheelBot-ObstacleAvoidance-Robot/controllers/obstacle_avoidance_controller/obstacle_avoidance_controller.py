# Import necessary modules from the Webots controller library
# This provides access to the Robot API and device interfaces
from controller import Robot, DistanceSensor, Motor

# Define the simulation step time in milliseconds
# This determines how frequently the controller is called
TIME_STEP = 64

# Define the maximum motor speed in radians per second
# This is used to calculate appropriate motor velocities
MAX_SPEED = 6.28

# Create an instance of the Robot class
# This represents the physical robot in the simulation
robot = Robot()

# Initialize an empty list to store distance sensor objects
ps = []
# Define names of the distance sensors as configured in the robot model
psNames = ["Rsensor", "Lsensor"]  # Right and Left sensors

# Initialize and enable each distance sensor
for i in range(2):
  # Get the sensor device by name from the robot
  ps.append(robot.getDevice(psNames[i]))
  # Enable the sensor with the specified sampling period
  ps[i].enable(TIME_STEP)

# Get motor devices by their names as configured in the robot model
# These represent the four motors of the 4-wheel robot
front_left_motor = robot.getDevice("LFmotor")  # Left Front motor
front_right_motor = robot.getDevice("RFmotor")  # Right Front motor
rear_left_motor = robot.getDevice("LBmotor")   # Left Back motor
rear_right_motor = robot.getDevice("RBmotor")  # Right Back motor

# Set position control to infinity for velocity control mode
# This allows us to control the motors by velocity rather than position
front_right_motor.setPosition(float('inf'))
front_left_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))

# Initialize all motor velocities to zero
front_right_motor.setVelocity(0.0)
front_left_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

# Define movement functions for different obstacle scenarios


def turn_right():
  """
  Make the robot turn right by setting opposite wheel directions.
  Returns the appropriate left and right wheel speeds.
  """
  rightSpeed = -0.8 * MAX_SPEED  # Right wheels move backward
  leftSpeed = 0.8 * MAX_SPEED    # Left wheels move forward
  return leftSpeed, rightSpeed


def turn_left():
  """
  Make the robot turn left by setting opposite wheel directions.
  Returns the appropriate left and right wheel speeds.
  """
  leftSpeed = -0.8 * MAX_SPEED   # Left wheels move backward
  rightSpeed = 0.8 * MAX_SPEED   # Right wheels move forward
  return leftSpeed, rightSpeed


def back_off():
  """
  Make the robot move backward by setting both wheels to reverse.
  Returns the appropriate left and right wheel speeds.
  """
  leftSpeed = -0.8 * MAX_SPEED   # Left wheels move backward
  rightSpeed = -0.8 * MAX_SPEED  # Right wheels move backward
  return leftSpeed, rightSpeed

#############################################################################


# Main control loop
while robot.step(TIME_STEP) != -1:  # Run until simulation is stopped

  # Read current values from the distance sensors
  psValues = []
  for i in range(2):
    psValues.append(ps[i].getValue())

  # Determine if obstacles are detected based on sensor thresholds
  # Lower values indicate closer obstacles (< 1000 means obstacle detected)
  right_obstacle = psValues[0] < 1000  # Right sensor detects obstacle
  left_obstacle = psValues[1] < 1000   # Left sensor detects obstacle

  # Set default motor speeds to 80% of maximum speed for forward movement
  left_speed = 0.8 * MAX_SPEED
  right_speed = 0.8 * MAX_SPEED

  # Obstacle avoidance logic
  if left_obstacle and right_obstacle:
    # If obstacles on both sides, first back off then turn right
    left_speed, right_speed = back_off()
    left_speed, right_speed = turn_right()
  elif left_obstacle:
    # If obstacle only on left side, turn right
    left_speed, right_speed = turn_right()
  elif right_obstacle:
    # If obstacle only on right side, turn left
    left_speed, right_speed = turn_left()
  # If no obstacles detected, continue with default forward movement

  # Apply the calculated speeds to all motors
  front_right_motor.setVelocity(right_speed)
  front_left_motor.setVelocity(left_speed)
  rear_left_motor.setVelocity(left_speed)
  rear_right_motor.setVelocity(right_speed)

#############################################################################
