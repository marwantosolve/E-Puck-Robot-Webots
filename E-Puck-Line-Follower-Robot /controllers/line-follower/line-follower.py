from controller import Robot, Motor, DistanceSensor

# Create the Robot instance
robot = Robot()

# Time step for the simulation (in milliseconds)
TIME_STEP = 64

# Get the motor devices
motor_right = robot.getDevice("right wheel motor")
motor_left = robot.getDevice("left wheel motor")

# Get the distance sensor devices
right_sensor = robot.getDevice("right_sensor")
left_sensor = robot.getDevice("left_sensor")

# Enable distance sensors with the defined time step
right_sensor.enable(TIME_STEP)
left_sensor.enable(TIME_STEP)

# Set the motors to rotate continuously
motor_right.setPosition(float('inf'))
motor_left.setPosition(float('inf'))

# Set initial speed
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)

# Define line following parameters
MAX_SPEED = 6.28      # Maximum speed (rad/s)
# Base speed for moving forward (reduced for better tracking)
BASE_SPEED = 2.0
# Proportional control coefficient (increased for sharper turns)
KP = 5.0

# For IR sensors on black/white surface:
# Lower values usually indicate darker surface (black line)
# Higher values usually indicate lighter surface (white floor)
# This might need tuning based on actual sensor readings
THRESHOLD = 500       # Initial threshold to detect the line

# Wait for sensor initialization and perform calibration
print("Initializing sensors and performing calibration...")
sum_left = 0
sum_right = 0
samples = 20

for i in range(samples):
  robot.step(TIME_STEP)
  sum_left += left_sensor.getValue()
  sum_right += right_sensor.getValue()

avg_left = sum_left / samples
avg_right = sum_right / samples
print(f"Average sensor values - Left: {avg_left:.2f}, Right: {avg_right:.2f}")

# Dynamically set threshold based on calibration
# The black line should give significantly lower values than the white floor
# Adding a bias to make it more sensitive to the black line
# Using 90% of average as threshold
THRESHOLD = (avg_left + avg_right) / 2 * 0.9

print(f"Set threshold to: {THRESHOLD:.2f}")
print("Starting line following...")

# Tracking variables for line detection reliability
last_line_seen = 0
MAX_LOST_TIME = 25  # Number of steps to continue searching before giving up

# Main control loop
while robot.step(TIME_STEP) != -1:
  # Read the sensor values
  left_ir = left_sensor.getValue()
  right_ir = right_sensor.getValue()

  # Print sensor values for debugging
  print(
      f"Left: {left_ir:.2f}, Right: {right_ir:.2f}, Threshold: {THRESHOLD:.2f}")

  # Line following logic:
  # For IR sensors, typically lower values indicate the dark line
  # Higher values indicate the light floor
  line_detected = False

  if left_ir < THRESHOLD or right_ir < THRESHOLD:
    # We see the line with at least one sensor
    last_line_seen = 0
    line_detected = True

    if left_ir < THRESHOLD and right_ir < THRESHOLD:
      # Both sensors on the line - move forward
      left_speed = BASE_SPEED
      right_speed = BASE_SPEED
      print("Both sensors on line - moving forward")
    elif left_ir < THRESHOLD and right_ir >= THRESHOLD:
      # Left sensor on the line, right sensor off - turn left
      left_speed = BASE_SPEED - KP
      right_speed = BASE_SPEED + KP
      print("Turning left")
    else:  # right_ir < THRESHOLD and left_ir >= THRESHOLD
      # Right sensor on the line, left sensor off - turn right
      left_speed = BASE_SPEED + KP
      right_speed = BASE_SPEED - KP
      print("Turning right")
  else:
    # No sensors directly on line - search pattern
    last_line_seen += 1

    if last_line_seen > MAX_LOST_TIME:
      # If we've been searching too long, slow down
      left_speed = BASE_SPEED * 0.5
      right_speed = 0  # Sharper turn
      print("Searching aggressively - line lost for too long")
    else:
      # Standard search pattern - making a wider slow turn to find the line
      left_speed = BASE_SPEED
      right_speed = BASE_SPEED * 0.3  # More aggressive right turn to search
      print("Searching - line temporarily lost")

  # Ensure speeds are within limits
  left_speed = max(0, min(left_speed, MAX_SPEED))
  right_speed = max(0, min(right_speed, MAX_SPEED))

  # Set motor speeds
  motor_left.setVelocity(left_speed)
  motor_right.setVelocity(right_speed)

  # Adaptive threshold adjustment based on readings
  if line_detected:
    # Slowly adapt the threshold based on actual readings
    line_value = min(left_ir, right_ir) if left_ir < right_ir else right_ir
    floor_value = max(left_ir, right_ir) if left_ir < right_ir else left_ir

    # Only update if there's a clear difference between line and floor
    if floor_value > line_value * 1.5:
      new_threshold = (line_value + floor_value) / 2
      # Smooth the threshold adaptation (slow change)
      THRESHOLD = THRESHOLD * 0.95 + new_threshold * 0.05
