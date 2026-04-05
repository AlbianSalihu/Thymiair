"""
motion.py — Motor control and finite-state machine for Thymio navigation.

FSM states: IDLE → Global (follow planned path) → Local (reactive obstacle avoidance)

Functions:
    motors          — build the tdmclient motor variable dict for left/right speeds
    compute_angle   — compute the signed angle the Thymio must rotate to face the next waypoint
    turn            — async: rotate in place until the target heading is reached
    go_to_position  — async: drive toward a waypoint, switching to local avoidance if blocked
    prox_trigger    — async: return True if any front proximity sensor exceeds the threshold

Note: `node` and `client` are tdmclient globals that must be set up in main.py before
calling the async functions.
"""

import cv2
import numpy as np

import vision
import kalman


def motors(right, left):
    """Return the tdmclient variable dict to set both motor targets.

    :param right: right motor speed (Thymio units, signed)
    :param left: left motor speed (Thymio units, signed)
    :returns: dict suitable for node.send_set_variables()
    """
    return {
        "motor.right.target": [right],
        "motor.left.target":  [left],
    }


def compute_angle(thymio_orientation, next_corner, thymio_position):
    """Compute the signed angle the Thymio must rotate to face next_corner.

    :param thymio_orientation: current heading in radians
    :param next_corner: [x, y] target waypoint
    :param thymio_position: [x, y] current Thymio position
    :returns: (delta_angle, anticlockwise, thymio_orientation)
    """
    EPS     = 0.1
    X_VALUE = 0
    Y_VALUE = 1
    vector_pos = next_corner - thymio_position
    if (np.linalg.norm(vector_pos) < EPS) and (np.linalg.norm(vector_pos) > -EPS):
        delta_angle = 0
    else:
        delta_angle = np.arctan2(vector_pos[Y_VALUE], vector_pos[X_VALUE]) - thymio_orientation

    anticlockwise = delta_angle < 0
    return delta_angle, anticlockwise, thymio_orientation


async def turn(delta_angle, anticlockwise, thymio_orientation, x_est, P_est, camera, cam_flag, T_S, vid):
    """Rotate the Thymio in place until the EKF heading reaches the target angle.

    :param delta_angle: signed angle to rotate [rad]
    :param anticlockwise: True for counter-clockwise rotation
    :param thymio_orientation: heading at the start of the turn [rad]
    :param x_est: (3,1) current EKF state
    :param P_est: (3,3) current EKF covariance
    :param camera: latest [x, y, theta] camera measurement
    :param cam_flag: 1 if camera unoccluded
    :param T_S: control loop period [s]
    :param vid: cv2.VideoCapture handle for the overhead camera
    :returns: (x_est, P_est) updated after the turn
    """
    Motor_speed        = 60
    CAM_FLAG           = 0
    CAM_THYMIO_CENTER  = 2
    CAM_THYMIO_CENTER_X = 0
    CAM_THYMIO_CENTER_Y = 1
    CAM_THYMIO_ANGLE   = 1
    Speed_null         = 0

    if anticlockwise:
        while x_est[2] > delta_angle + thymio_orientation:
            ret, frame = vid.read()
            frame_HSV  = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            info_thym  = vision.find_thymio(frame_HSV)
            cam_flag   = info_thym[CAM_FLAG]
            camera     = [info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_X],
                          info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_Y],
                          info_thym[CAM_THYMIO_ANGLE]]
            x_est, P_est = kalman.kalman_filter(Motor_speed, -Motor_speed, camera, x_est, P_est, cam_flag)
            node.send_set_variables(motors(Motor_speed, -Motor_speed))
            await client.sleep(T_S / 3)
    else:
        while x_est[2] < delta_angle:
            ret, frame = vid.read()
            frame_HSV  = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            info_thym  = vision.find_thymio(frame_HSV)
            cam_flag   = info_thym[CAM_FLAG]
            camera     = [info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_X],
                          info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_Y],
                          info_thym[CAM_THYMIO_ANGLE]]
            x_est, P_est = kalman.kalman_filter(-Motor_speed, Motor_speed, camera, x_est, P_est, cam_flag)
            node.send_set_variables(motors(-Motor_speed, Motor_speed))
            await client.sleep(T_S / 3)

    node.send_set_variables(motors(Speed_null, Speed_null))
    await client.sleep(T_S / 3)
    return x_est, P_est


async def go_to_position(corner_pos, x_est, P_est, camera, cam_flag, T_S, vid, EPS=3):
    """Drive the Thymio toward corner_pos, engaging reactive avoidance if a sensor fires.

    :param corner_pos: [x, y] target waypoint in image coordinates
    :param x_est: (3,1) current EKF state
    :param P_est: (3,3) current EKF covariance
    :param camera: latest [x, y, theta] camera measurement
    :param cam_flag: 1 if camera unoccluded
    :param T_S: control loop period [s]
    :param vid: cv2.VideoCapture handle for the overhead camera
    :param EPS: positional tolerance in pixels (default 3)
    :returns: (x_est, P_est, obstacle) — obstacle=True if avoidance was triggered
    """
    Motor_speed          = 60
    Threshold            = 2000
    Prox_sensor_left     = 0
    Prox_sensor_left_middle  = 1
    Prox_sensor_middle   = 2
    Prox_sensor_right_middle = 3
    Prox_sensor_right    = 4
    CAM_FLAG             = 0
    CAM_THYMIO_CENTER    = 2
    CAM_THYMIO_CENTER_X  = 0
    CAM_THYMIO_CENTER_Y  = 1
    CAM_THYMIO_ANGLE     = 1
    X_VALUE              = 0
    Y_VALUE              = 1

    delta_X  = float(np.abs(-x_est[X_VALUE] + corner_pos[X_VALUE]))
    delta_Y  = float(np.abs(-x_est[Y_VALUE] + corner_pos[Y_VALUE]))
    obstacle = False

    while (delta_X > EPS) and (delta_Y > EPS):
        ret, frame = vid.read()
        frame_HSV  = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        info_thym  = vision.find_thymio(frame_HSV)

        cam_flag = info_thym[CAM_FLAG]
        camera   = [info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_X],
                    info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_Y],
                    info_thym[CAM_THYMIO_ANGLE]]
        x_est, P_est = kalman.kalman_filter(Motor_speed, Motor_speed, camera, x_est, P_est, cam_flag)
        delta_X  = float(np.abs(-x_est[X_VALUE] + corner_pos[X_VALUE]))
        delta_Y  = float(np.abs(-x_est[Y_VALUE] + corner_pos[Y_VALUE]))

        node.send_set_variables(motors(Motor_speed, Motor_speed))
        await client.sleep(T_S / 3)

        prox = await node.wait_for_variables({"prox.horizontal"})
        proximity_left  = False
        proximity_right = False
        motor_left      = Motor_speed
        motor_right     = Motor_speed

        while (await prox_trigger(T_S)):
            obstacle = True
            await node.wait_for_variables({"prox.horizontal"})
            await client.sleep(T_S)

            if (node.v.prox.horizontal[Prox_sensor_left]        > Threshold or
                node.v.prox.horizontal[Prox_sensor_left_middle] > Threshold or
                node.v.prox.horizontal[Prox_sensor_middle]      > Threshold):
                proximity_left = True
            elif (node.v.prox.horizontal[Prox_sensor_right_middle] > Threshold or
                  node.v.prox.horizontal[Prox_sensor_right]         > Threshold):
                proximity_right = True

            if proximity_right:
                motor_left  = Motor_speed
                motor_right = -Motor_speed
            elif proximity_left:
                motor_left  = -Motor_speed
                motor_right = Motor_speed
            else:
                motor_left  = Motor_speed
                motor_right = Motor_speed

            ret, frame = vid.read()
            frame_HSV  = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            info_thym  = vision.find_thymio(frame_HSV)
            cam_flag   = info_thym[CAM_FLAG]
            camera     = [info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_X],
                          info_thym[CAM_THYMIO_CENTER][CAM_THYMIO_CENTER_Y],
                          info_thym[CAM_THYMIO_ANGLE]]

            x_est, P_est = kalman.kalman_filter(motor_left, motor_right, camera, x_est, P_est, cam_flag)
            node.send_set_variables(motors(motor_left, motor_right))
            await client.sleep(T_S)

        if obstacle:
            break

    return x_est, P_est, obstacle


async def prox_trigger(T_S):
    """Return True if any front proximity sensor reading exceeds the obstacle threshold.

    :param T_S: sleep duration [s] applied after a trigger is detected
    :returns: True if an obstacle is sensed, False otherwise
    """
    Threshold            = 2000
    Prox_sensor_left     = 0
    Prox_sensor_left_middle  = 1
    Prox_sensor_middle   = 2
    Prox_sensor_right_middle = 3
    Prox_sensor_right    = 4
    trigger              = False

    if (node.v.prox.horizontal[Prox_sensor_left]         >= Threshold or
        node.v.prox.horizontal[Prox_sensor_left_middle]  >= Threshold or
        node.v.prox.horizontal[Prox_sensor_middle]        >= Threshold or
        node.v.prox.horizontal[Prox_sensor_right_middle] >= Threshold or
        node.v.prox.horizontal[Prox_sensor_right]         >= Threshold):
        trigger = True
        await client.sleep(T_S)

    return trigger
