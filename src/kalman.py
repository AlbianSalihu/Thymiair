"""
kalman.py — Extended Kalman Filter for fusing wheel odometry and overhead camera pose.

The EKF uses a linearized unicycle motion model.  When the camera is unoccluded
(cam_flag=1) it performs a full predict+correct step; when occluded it runs
predict-only, coasting on odometry alone.
"""

import numpy as np


def kalman_filter(left_speed, right_speed, camera, x_est_prev, P_est_prev, cam_flag,
                  HT=None, HNT=None, RT=None, RNT=None):
    """Run one EKF step: predict from wheel odometry, then correct with camera if available.

    :param left_speed: left motor speed in Thymio units
    :param right_speed: right motor speed in Thymio units
    :param camera: [x, y, theta] pose measurement from the overhead camera
    :param x_est_prev: (3,1) previous a-posteriori state estimate [x, y, theta]
    :param P_est_prev: (3,3) previous a-posteriori covariance matrix
    :param cam_flag: 1 if camera is unoccluded (correction applied), 0 if occluded
    :returns: (x_est, P_est) updated state and covariance
    """
    # Tuned parameters
    q_px    = 2    # process noise variance on x
    q_py    = 2    # process noise variance on y
    q_alpha = 2    # process noise variance on heading
    r_px    = 0.1  # camera measurement noise variance on x
    r_py    = 0.1  # camera measurement noise variance on y
    r_alpha = 0.1  # camera measurement noise variance on heading
    Ts      = 0.1  # sample period [s]
    b       = 100  # wheel-base distance [mm]

    Q = np.array([[q_px, 0, 0], [0, q_py, 0], [0, 0, q_alpha]], dtype=float)

    speed_conv_factor = 0.4 * 84 / 50

    # Prediction — linearized unicycle motion model
    ds          = speed_conv_factor * (left_speed + right_speed) * Ts / 2
    delta_alpha = speed_conv_factor * (right_speed - left_speed) * Ts / b
    u           = np.array([[ds], [delta_alpha]])
    alpha_est_prev = x_est_prev[2]

    camera_x     = camera[0]
    camera_y     = camera[1]
    camera_alpha = camera[2]

    A = np.array([[1, 0, -ds * np.sin((alpha_est_prev + delta_alpha / 2))],
                  [0, 1,  ds * np.cos((alpha_est_prev + delta_alpha / 2))],
                  [0, 0,  1]], dtype=float)

    B = np.array([[np.cos((alpha_est_prev + delta_alpha / 2)),
                   -ds / 2 * np.sin((alpha_est_prev + delta_alpha / 2))],
                  [np.sin((alpha_est_prev + delta_alpha / 2)),
                    ds / 2 * np.cos((alpha_est_prev + delta_alpha / 2))],
                  [0, 1]], dtype=float)

    x_est = A @ x_est_prev + B @ u
    P_est = np.dot(A, np.dot(P_est_prev, A.T)) + Q

    # Correction — only when camera is unoccluded
    if cam_flag:
        z = np.array([[camera_x], [camera_y], [camera_alpha]])
        H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        R = np.array([[r_px, 0, 0], [0, r_py, 0], [0, 0, r_alpha]])

        i = z - np.dot(H, x_est)          # innovation
        S = np.dot(H, np.dot(P_est, H.T)) + R
        K = np.dot(P_est, np.dot(H.T, np.linalg.inv(S)))  # Kalman gain

        x_est = x_est + np.dot(K, i)
        P_est = P_est - np.dot(K, np.dot(H, P_est))

    return x_est, P_est
