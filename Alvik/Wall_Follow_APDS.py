from arduino_alvik import ArduinoAlvik
from machine import I2C, Pin
import time

# --- Initialize Alvik ---
alvik = ArduinoAlvik()
alvik.begin()

# --- Initialize APDS-9960 sensor on I2C ---
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
APDS_ADDR = 0x39

ENABLE  = 0x80
ATIME   = 0x81
CONTROL = 0x8F
PDATA   = 0x9C  # Proximity data register

def apds_init():
    i2c.writeto_mem(APDS_ADDR, ENABLE, b'\x0F')
    i2c.writeto_mem(APDS_ADDR, ATIME, b'\xFF')
    i2c.writeto_mem(APDS_ADDR, CONTROL, b'\x01')

def read_proximity():
    return int.from_bytes(i2c.readfrom_mem(APDS_ADDR, PDATA, 1), 'little')

apds_init()

# --- Wall-follow parameters ---
TARGET =30      # desired distance to the left wall
Kp = 1.2         # proportional gain
BASE = 50       # base speed

# --- Front obstacle parameters ---
OBSTACLE_DIST = 10 # mm, stop if closer than this
##----wall follower function
def wall():
   # --- Wall-follow P-controller ---
    error = TARGET - prox
    correction = int(Kp * error)

    left_speed = BASE - correction
    right_speed = BASE + correction

    # Clamp speeds
    left_speed = max(min(left_speed, 80), -80)
    right_speed = max(min(right_speed, 80), -80)

    # # --- Front obstacle check ---
    # if front_dist < OBSTACLE_DIST:
    #     left_speed = 0
    #     right_speed = 0
    #     print("Obstacle ahead! Stopping...")

    # --- Set wheel speeds ---
    alvik.set_wheels_speed(left_speed, right_speed)

    # --- Debug print ---
    print(f"Prox:{prox}  Front:{front_dist}  Error:{error}  L:{left_speed} R:{right_speed}")

def turn(degrees, direction='right'):
    """
    Rotates the robot by a specified number of degrees.
    direction: 'right' for clockwise, 'left' for counter-clockwise
    """
    if direction == 'left':
        degrees = -abs(degrees)
    else:
        degrees = abs(degrees)

    print(f"Turning {direction} by {abs(degrees)}°...")
    alvik.rotate(angle=degrees, unit='deg', blocking=True)
    print("Turn complete.")

def move_forward_cm(cm, speed=10):
    """
    Moves Alvik forward by a specified distance in cm.
    speed: linear velocity in cm/s
    """
    duration = cm / speed  # time in seconds
    alvik.drive(linear_velocity=speed, angular_velocity=0)
    time.sleep(duration)
    alvik.brake()
    print(f"Moved forward {cm} cm.")
def sweeping_turn(forward_speed=10, turn_rate=30, duration=1.5):
    """
    Makes Alvik perform a sweeping turn.
    forward_speed: cm/s forward motion
    turn_rate: deg/s angular velocity (positive = right, negative = left)
    duration: seconds to maintain the curve
    """
    alvik.drive(linear_velocity=forward_speed, angular_velocity=turn_rate)
    time.sleep(duration)
    alvik.brake()
    print(f"Sweeping turn complete: {turn_rate}°/s for {duration}s")
def check_color_action():
    """
    Reads the current color label from Alvik’s sensor
    and takes action based on the detected color.
    """
    label = alvik.get_color_label()      # e.g. "RED", "GREEN", "BLUE", etc.
    print(f"Detected color: {label}")
    return label
    
    # if label == "BLACK":
    #     alvik.brake()
    #     print("BLACK detected — stopping.")
    #     # return True                     # signal to exit or change mode
    # elif label == "BLUE":
    #     # Example: sweeping right turn on blue
    #     sweeping_turn(forward_speed=10, turn_rate=30, duration=1.5)
    # # add more color‐based behaviors as needed
    
    # return False


while True:
    # --- Read sensors ---
    prox = read_proximity()                 # left wall distance
    front_dist = alvik.get_distance()[2]    # center zone of built-in ToF sensor (front)
    col = check_color_action()
    while col not in ['BLUE','BLACK','GREY']:
      
    # turn(90,'left')
    # wall() 
    # print(f'Dist:{prox,front_dist}')
      if prox <100 and front_dist<12:
        turn(90,'left')#right and left inverted
        time.sleep(.5)
      elif prox<100 and front_dist>15:
        wall()
    # # elif prox>100 and front_dist>15:
    # #   move_forward_cm(2)
    # #   sweeping_turn()
      else:
        wall()
    if col == 'BLACK':
      alvik.brake()
      turn(180)
    elif col == 'BLUE':
      alvik.brake()
      time.sleep(5)
      while col =='BLUE':
        alvik.drive(50,50)
  
        
       
    
   
    # time.sleep(0.1)
