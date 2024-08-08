import math
import numpy as np

def reward_function1(params):
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

    if not( steering==0 and direction_diff==0) and speed > 2.5 - (0.4 * steering):
        reward *= 0.8

    
    
    return float(reward)

def calculate_heading_angle(p1, p2):
    """
    Calculate the angle between two points.
    p1: tuple (x1, y1)
    p2: tuple (x2, y2)
    Returns the angle in degrees.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.arctan2(dy, dx) * (180 / np.pi)

def check_if_in_turn(waypoints,closest_waypoints):
    
    prev_point = waypoints[closest_waypoints[0]]
    next_point = waypoints[closest_waypoints[1]]
    next_next_point = waypoints[(closest_waypoints[1]+1)%len(waypoints)]
    vec_p1_p2 = np.array(next_point) - np.array(prev_point)
    vec_p1_p3 = np.array(next_next_point) - np.array(prev_point)

    # Calculate the angle between the vectors
    angle_between = np.degrees(np.arctan2(vec_p1_p2[1], vec_p1_p2[0]) - np.arctan2(vec_p1_p3[1], vec_p1_p3[0]))
    angle_between = (angle_between + 180) % 360 - 180  # Normalize to -180 to 180

    # Check if the angle is significant enough to indicate being in a turn
    return abs(angle_between) > 15 


def is_start_or_end_of_turn(current_pos, waypoints,closest_waypoints, current_heading):
    """
    Check if the car is at the start or end of a turn and the direction of the turn.
    
    current_pos: tuple (x, y) of the current position of the vehicle.
    waypoints: list of tuples representing the track waypoints.
    current_heading: heading of the vehicle in degrees.
    
    Returns:
        'start-left' if at the start of a left turn,
        'start-right' if at the start of a right turn,
        'end' if at the end of a turn,
        'none' if neither.
    """
    if len(waypoints) < 3:
        return 'none'
    
    # Get the next two waypoints
    next_point = waypoints[closest_waypoints[1]]
    next_next_point = waypoints[(closest_waypoints[1]+1)%len(waypoints)]

    # Calculate the angle to the next waypoint
    angle_to_next = calculate_heading_angle(current_pos, next_point)

    # Calculate the turn direction
    turn_angle = angle_to_next - current_heading

    # Normalize the angle to be within -180 to 180
    turn_angle = (turn_angle + 180) % 360 - 180

    # Check if the car is starting to turn or has completed the turn
    if abs(turn_angle) < 15:  # Close to heading
        return 'none'  # Not in a significant turn

    # Determine if at the start of the turn
    if turn_angle > 0:  # Right turn
        if current_heading < angle_to_next and abs(turn_angle) > 15:
            return 'start-right'
    else:  # Left turn
        if current_heading > angle_to_next and abs(turn_angle) > 15:
            return 'start-left'
    
    # Determine if at end of the turn
    angle_to_next_next = calculate_heading_angle(next_point, next_next_point)
    if abs(angle_to_next_next - current_heading) < 15:  # Close to the next next waypoint heading
        if turn_angle > 0:  # Right turn
            return 'end-right'
        else:  # Left turn
            return 'end-left'

    return 'none'



def calculate_turn_radius(p1, p2, p3):
    """
    Calculate the radius of curvature given three points.
    p1: Current position (x, y)
    p2: Next waypoint (x, y)
    p3: Waypoint after next (x, y)
    Returns the radius of curvature.
    """
    # Unpack points
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    # Calculate distances
    a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)  # Distance between p1 and p2
    b = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)  # Distance between p2 and p3
    c = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)  # Distance between p1 and p3
    
    # Calculate semi-perimeter
    s = (a + b + c) / 2
    
    # Calculate area using Heron's formula
    area = np.sqrt(s * (s - a) * (s - b) * (s - c))
    
    # Radius of curvature
    radius = (a * b * c) / (4 * area) if area != 0 else float('inf')
    
    return radius

def is_turn_upcoming(closest_waypoints, waypoints, turn_radius_threshold=5.0):
    """
    Check if a turn is upcoming based on waypoints.
    
    current_pos: tuple (x, y) of the current position of the vehicle.
    waypoints: list of tuples representing the track waypoints.
    turn_radius_threshold: threshold for determining if a turn is sharp.
    Returns a tuple (bool, str) indicating if a turn is upcoming and its direction.
    """
    # Ensure there are enough waypoints to analyze
    if len(waypoints) < 3:
        return False, None
    
    # Get the current position
    p1 = waypoints[closest_waypoints[1]]
    p2 = waypoints[(closest_waypoints[1]+1)%len(waypoints)]  # Next waypoint
    p3 = waypoints[(closest_waypoints[1]+2)%len(waypoints)] # Waypoint after next
    
    # Calculate turn radius
    turn_radius = calculate_turn_radius(p1, p2, p3)
    
    # Determine the direction of the turn
    angle_to_next = calculate_heading_angle(p1, p2)
    angle_to_next_next = calculate_heading_angle(p2, p3)
    
    turn_direction = 'left' if (angle_to_next_next - angle_to_next) < 0 else 'right'
    
    # Check if the turn radius is less than the threshold
    return turn_radius < turn_radius_threshold, turn_direction

def reward_function(params):
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle'])
    is_left = params['is_left_of_center']
    current_pos=[params['x'],params['y']]

    reward = 1.0

    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    if not all_wheels_on_track:
                    reward = 1e-3
    if check_if_in_turn(waypoints,closest_waypoints):
        turn_pos= is_start_or_end_of_turn(current_pos,waypoints,closest_waypoints,heading)
        if turn_pos == 'none':
                if distance_from_center <= marker_1:
                    reward *= 1.2
                elif distance_from_center <= marker_2:
                    reward *= 0.8
                elif distance_from_center <= marker_3:
                    reward *= 0.5
                else:
                    reward = 1e-3

                
        elif turn_pos == 'start-right':
            if is_left:
                if distance_from_center <= marker_1:
                    reward *= 0.7
                elif distance_from_center <= marker_2:
                    reward *= 0.9
                elif distance_from_center <= marker_3:
                    reward *= 1.2
                else:
                    reward = 1e-3
            else:
                reward *=0.5
        elif turn_pos == 'start-left':
            if not is_left:
                if distance_from_center <= marker_1:
                    reward *= 0.6
                elif distance_from_center <= marker_2:
                    reward *= 0.8
                elif distance_from_center <= marker_3:
                    reward *= 1.2
                else:
                    reward = 1e-3
            else:
                reward *=0.5
        elif turn_pos == 'end-left':
            if  is_left:
                if distance_from_center <= marker_1:
                    reward *= 0.7
                elif distance_from_center <= marker_2:
                    reward *= 0.9
                elif distance_from_center <= marker_3:
                    reward *= 1.1
                else:
                    reward = 1e-3
            else:
                reward *=0.5
        elif turn_pos == 'end-right':
            if not is_left:
                if distance_from_center <= marker_1:
                    reward *= 0.7
                elif distance_from_center <= marker_2:
                    reward *= 0.9
                elif distance_from_center <= marker_3:
                    reward *= 1.1
                else:
                    reward = 1e-3
            else:
                reward *=0.5  
    else:
        bool_turn, lr = is_turn_upcoming(closest_waypoints, waypoints) 
        if bool_turn:
            if lr=='left':
                if not is_left:
                    if distance_from_center <= marker_1:
                        reward *= 0.7
                    elif distance_from_center <= marker_2:
                        reward *= 0.9
                    elif distance_from_center <= marker_3:
                        reward *= 1.1
                    else:
                        reward = 1e-3
                else:
                    reward *=0.5 

            else:
                if  is_left:
                    if distance_from_center <= marker_1:
                        reward *= 0.7
                    elif distance_from_center <= marker_2:
                        reward *= 0.9
                    elif distance_from_center <= marker_3:
                        reward *= 1.1
                    else:
                        reward = 1e-3
                else:
                    reward *=0.5
        else:
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

            if not( steering==0 and direction_diff==0) and speed > 2.5 - (0.4 * steering):
                reward *= 0.8

    return float(reward)


            
                






