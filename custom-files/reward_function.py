import math

def reward_function(params):
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle'])

    reward = 1.0

    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    if distance_from_center <= marker_1:
        reward *= 1.2
    elif distance_from_center <= marker_2:
        reward *= 0.8
    elif distance_from_center <= marker_3:
        reward *= 0.5
    else:
        reward = 1e-3

    if not all_wheels_on_track:
        reward = 1e-3

    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_direction = math.degrees(track_direction)
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    DIRECTION_THRESHOLD = 10.0
    if direction_diff > DIRECTION_THRESHOLD:
        reward *= 0.5

    ABS_STEERING_THRESHOLD = 30
    if steering > ABS_STEERING_THRESHOLD:
        reward *= 0.8

    if direction_diff < 5 and speed > 3:
        reward *= 1.2

    if steering>5 and speed > 3.5 - (0.4 * steering):
        reward *= 0.8
    
    return float(reward)
