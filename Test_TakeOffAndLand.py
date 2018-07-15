from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default(24.180956, 120.649184)
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
print("(Connecting to vehicle on: /dev/ttyAMA0)")
f.write("\n Connecting to vehicle on: %s" % connection_string)
f.write("\n (Connecting to vehicle on: /dev/ttyAMA0)")
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
# vehicle = connect(connection_string, wait_ready=True, baud=921600)


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
    """
        Arms vehicle and fly to aTargetAltitude.
        """
    print("\n Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("\n  Waiting for vehicle to initialise...")
        time.sleep(1)

    print("\n Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("\n  Waiting for arming...")
        time.sleep(1)

    print("\n Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("\n  Altitude: %s " % vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("\n Reached target altitude")
            break
        time.sleep(1)


# Initialize the takeoff sequence to 10m
arm_and_takeoff(10)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
