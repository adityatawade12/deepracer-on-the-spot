import mat

 

def reward_function(params):  

    #distnce from centre
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    
    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    
    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.7
    elif distance_from_center <= marker_3:
        reward = 0.4
    else:
        reward = 1e-3 

    #reward in angle
    # Read input variables

    waypoints = params['waypoints']

    closest_waypoints = params['closest_waypoints']

    heading = params['heading']

 

    # Initialize the reward with typical value

    # reward = 1.0

    # Calculate the direction of the center line based 

    next_point = waypoints[closest_waypoints[1]]

    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), 

    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Convert to degree

    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car

    direction_diff = abs(track_direction - heading)

    if direction_diff > 180:

        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large

    DIRECTION_THRESHOLD = 10.0

    if direction_diff > 20:
        reward *= 0.1
    elif direction_diff > 10:
        reward *=0.5
    elif direction_diff > 5:
        reward *=0.8
    elif direction_diff == 0:
        reward *=1.2

    
   

    

    return float(reward)

