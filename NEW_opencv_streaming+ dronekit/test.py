#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from feature_matching import FeatureMatching
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import threading
import time
import math
from math import *
import traceback

traceback.print_exc()

# Create a timestamped file to log the data
timestr = time.strftime("%Y_%m%d-%H_%M_%S")
filename = "flight_" + timestr + ".txt"
f = open(filename, "w+")

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
f.write("\n Connecting to vehicle on: %s" % connection_string)
print("(Connecting to vehicle on: /dev/ttyAMA0)")
f.write("\n (Connecting to vehicle on: /dev/ttyAMA0)")
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
# vehicle = connect(connection_string, wait_ready=True, baud=921600)


# check it real reaching
def goto(gps_location, UpOrDown):
    # print("\n Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")

    while not vehicle.armed:
        # print("\n  Waiting for arming...")
        time.sleep(1)

    vehicle.airspeed = 3
    gps_location = LocationGlobalRelative(gps_location.lat, gps_location.lon, gps_location.alt)
    vehicle.simple_goto(gps_location, groundspeed=5)
    vehicle.flush()

    while True:
        print("correct goal position...")
        f.write("\n correct goal position...")

        # Limit latitude and longitude and height within a certain range
        if (vehicle.location.global_relative_frame.alt >= gps_location.alt * 0.99 and vehicle.location.global_relative_frame.alt <= gps_location.alt * 1.01) and \
                (vehicle.location.global_relative_frame.lon >= gps_location.lon * 0.9999996 and vehicle.location.global_relative_frame.lon <= gps_location.lon * 1.0000003) and \
                (vehicle.location.global_relative_frame.lat >= gps_location.lat * 0.99996 and vehicle.location.global_relative_frame.lat <= gps_location.lat * 1.00003):
            break
        time.sleep(2)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("\n Basic pre-arm checks")
    f.write("\n\n Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("\n  Waiting for vehicle to initialise...")
        f.write("\n\n Waiting for vehicle to initialise...")
        time.sleep(1)

    print("\n Arming motors")
    f.write("\n\n Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.flush()

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("\n  Waiting for arming...")
        f.write("\n\n Waiting for arming...")
        time.sleep(1)

    print("\n Taking off!")
    f.write("\n\n Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("\n  Altitude: %s " % vehicle.location.global_relative_frame.alt)
        f.write("\n\n  Altitude: %s " % vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.99 and vehicle.location.global_relative_frame.alt <= aTargetAltitude * 1.01:
            print("\n Reached target altitude")
            f.write("\n\n Reached target altitude")
            break
        time.sleep(1)


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading
    (in degrees).
    This method sets an absolute heading by default, but you can set the
    `relative` parameter to `True` to set yaw relative to the current yaw
    heading.
    By default the yaw of the vehicle will follow the direction of travel.
    After setting the yaw using this function there is no way to return to the
    default yaw "follow direction of travel" behaviour
    (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    # This makes yawing work
    # send_global_velocity(0, 0, 0)

    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        abs(heading),  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        math.copysign(1, heading),  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


# check its head face north
def real_condition_yaw():
    # print("\n Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")

    while not vehicle.armed:
        # print("\n  Waiting for arming...")
        time.sleep(1)

    condition_yaw(0)
    print("Velocity North")
    f.write("\n Velocity North")
    send_ned_velocity(0, 0, 0, 1)

    while True:
        print("Let the head face north...")
        f.write("\n Let the head face north...")
        if (vehicle.heading == 0):
            break
        time.sleep(2)

    print("Yaw 0 absolute (North)")
    f.write("\n Yaw 0 absolute (North)\n\n")


# main
# Set altitude to 20 meters above the current altitude
arm_and_takeoff(20)

print("\n Set default/target airspeed to 3.")
f.write("\n\n Set default/target airspeed to 3.")
vehicle.airspeed = 3

print("\n Set groundspeed to 5m/s.")
f.write("\n\n Set groundspeed to 5m/s.")
vehicle.groundspeed = 5
DURATION = 20  # Set duration for each segment.
NORTH = 2  # vx > 0 => fly North

loc = vehicle.location.global_relative_frame  # get current location

# If it's a real mission , that you can fly a path using specific GPS coordinates.
print("\n Going to GOAL Position")
f.write("\n\n Going to GOAL Position\n")
# vehicle.simple_goto(lat, lon, alt)
# time.sleep(20)

print("\n start correcting goal position:\n")
f.write("\n start correcting goal position:\n")
correct = 0
times = 0
GoalNum = 1

try:
    VideoStream = cv2.VideoCapture(0)  # index of your camera
    VideoStream.set(3, 1920)
    VideoStream.set(4, 1080)
except:
    print "problem opening input stream"
    f.write("\n problem opening input stream")

FailTime = 0
while (VideoStream.isOpened()):
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Still can not find the goal
    if (correct == -10):
        vehicle.mode = VehicleMode("RTL")
        print("Returning to Launch")
        f.write("\n\n Returning to Launch")
        print("\n Failed")
        f.write("\n\n Failed")
        break

    # check until plane land
    if (loc.alt == 0):
        vehicle.mode = VehicleMode("LAND")
        print("LAND")
        f.write("\n LAND")
        print("\n Completed")
        f.write("\n\n Completed")
        break

    # Let the head face north
    real_condition_yaw()

    while True:
        try:
            loc = vehicle.location.global_relative_frame  # get current location
            print("%s" % loc)
            f.write("%s\n\n" % loc)
            # Capture frame-by-frame
            ret, frame = VideoStream.read()
            if ret == True:
                if (GoalNum == 1):
                    # write the frame
                    timestr = time.strftime("%Y_%m%d-%H_%M_%S")
                    ImageName = "image_" + timestr + "__" + str(times + 1) + ".jpg"
                    cv2.imwrite(ImageName, frame)
                    print("\n save picture" + str(times + 1) + "...\n")
                    f.write("\n  save picture" + str(times + 1) + "...\n")

                    # camera's image
                    img_train = cv2.imread('./' + ImageName)
                    # goal image
                    print(ImageName + " compare to Goal_1.jpg")
                    f.write("\n " + ImageName + " compare to Goal_1.jpg")
                    query_image = cv2.imread('./Goal_1.jpg')
                    matching = FeatureMatching(query_image='./Goal_1.jpg')
                elif (GoalNum == 2):
                    print(ImageName + " compare to Goal_2.jpg")
                    f.write("\n " + ImageName + " compare to Goal_2.jpg")
                    query_image = cv2.imread('./Goal_2.jpg')
                    matching = FeatureMatching(query_image='./Goal_2.jpg')
                elif (GoalNum == 3):
                    print(ImageName + " compare to Goal_3.jpg")
                    f.write("\n " + ImageName + " compare to Goal_3.jpg")
                    query_image = cv2.imread('./Goal_3.jpg')
                    matching = FeatureMatching(query_image='./Goal_3.jpg')
                elif (GoalNum == 4):
                    print(ImageName + " compare to Goal_4.jpg")
                    f.write("\n " + ImageName + " compare to Goal_4.jpg")
                    query_image = cv2.imread('./Goal_4.jpg')
                    matching = FeatureMatching(query_image='./Goal_4.jpg')
                elif (GoalNum == 5):
                    print(ImageName + " compare to Goal_5.jpg")
                    f.write("\n " + ImageName + " compare to Goal_5.jpg")
                    query_image = cv2.imread('./Goal_5.jpg')
                    matching = FeatureMatching(query_image='./Goal_5.jpg')
                elif (GoalNum == 6):
                    print(ImageName + " compare to Goal_6.jpg")
                    f.write("\n " + ImageName + " compare to Goal_6.jpg")
                    query_image = cv2.imread('./Goal_6.jpg')
                    matching = FeatureMatching(query_image='./Goal_6.jpg')
                elif (GoalNum == 7):
                    print(ImageName + " compare to Goal_7.jpg")
                    f.write("\n " + ImageName + " compare to Goal_7.jpg")
                    query_image = cv2.imread('./Goal_7.jpg')
                    matching = FeatureMatching(query_image='./Goal_7.jpg')
                elif (GoalNum == 8):
                    print(ImageName + " compare to Goal_8.jpg")
                    f.write("\n " + ImageName + " compare to Goal_8.jpg")
                    query_image = cv2.imread('./Goal_8.jpg')
                    matching = FeatureMatching(query_image='./Goal_8.jpg')
                elif (GoalNum == 9):
                    print(ImageName + " compare to Goal_9.jpg")
                    f.write("\n " + ImageName + " compare to Goal_9.jpg")
                    query_image = cv2.imread('./Goal_9.jpg')
                    matching = FeatureMatching(query_image='./Goal_9.jpg')
                elif (GoalNum == 10):
                    print(ImageName + " compare to Goal_10.jpg")
                    f.write("\n " + ImageName + " compare to Goal_10.jpg")
                    query_image = cv2.imread('./Goal_10.jpg')
                    matching = FeatureMatching(query_image='./Goal_10.jpg')

                m, x, y = matching.match(img_train, query_image)
                break
        except:
            print("This picture cannot be taken any special point, and will be tried again.\n")
            f.write("\n This picture cannot be taken any special point, and will be tried again.\n")
            if (GoalNum < 10):
                GoalNum += 1
            else:
                GoalNum = 11
                break

    if (GoalNum == 11 or m == False):
        print("not match!!!\n")
        f.write("\n not match!!!\n")

        if (GoalNum < 10):
            GoalNum += 1
        else:
            GoalNum = 1
            times += 1

        loc = vehicle.location.global_relative_frame  # get current location
        # Because it's first time, maybe the goal not in the vision, so go higher!
        if (loc.alt >= 10):
            loc.alt = loc.alt - 1
            goto(loc)
            correct += -1

        FailTime += 1

    # got the goal position
    else:
        # Set mode to guided - this is optional as the simple_goto method will change the mode if needed.
        vehicle.mode = VehicleMode("GUIDED")
        # correct the position and reduce the altitude
        loc = vehicle.location.global_relative_frame  # get current location
        # Latitude: 1 deg = 110.574 km = 110.574 * 100000 cm
        # Longitude: 1 deg = 111.320*cos(latitude) km = 111.320*cos(latitude) * 100000 cm
        # 1 pixel = 0.02645833 cm
        loc.lat = loc.lat + ((y * 0.02645833) / (110.574 * 100000))
        loc.lon = loc.lon + ((x * 0.02645833) / (111.320 * cos(loc.lat) * 100000))
        loc.alt = loc.alt - 1
        goto(loc)

        print ">>correct:",
        f.write("\n >>correct:")
        if (x == 0 and y == 0):
            print "mission clear!!!\n"
            f.write("mission clear!!!\n")
        else:
            if (y < 0):
                print "south",
                f.write("south")
                if (x != 0):
                    print ", ",
                    f.write(", ")
            elif (y > 0):
                print "north",
                f.write("north")
                if (x != 0):
                    print ", ",
                    f.write(", ")
            if (x < 0):
                print("west\n")
                f.write("west\n")
            elif (x > 0):
                print("east\n")
                f.write("east\n")
            else:
                print("\n")
                f.write("\n")
        correct += 1
        times += 1
        GoalNum = 1

# Camera problem
if not VideoStream.isOpened():
    print("Pleace recheck your camera !!!")
    f.write("\n\n Pleace recheck your camera !!!")
    vehicle.mode = VehicleMode("RTL")
    print("Returning to Launch")
    f.write("\n\n Returning to Launch")

# When everything done, release the capture
VideoStream.release()
cv2.destroyAllWindows()

# Close vehicle object before exiting script
print " Close vehicle object"
f.write("\n\n Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

f.close()