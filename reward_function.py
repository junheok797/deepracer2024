import math

def reward_function(params):
    """ Read input parameters """
    all_wheels_on_track = params['all_wheels_on_track']
    current_pose_x = params['x']
    current_pose_y = params['y']
    closest_waypoint = params['closest_waypoints']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    is_reversed = params['is_reversed']
    heading = params['heading']
    progress = params['progress']
    speed = params['speed']
    steering_angle = params['steering_angle']
    steps = params['steps']
    track_length = params['track_length']
    track_width = params['track_width']
    waypoints = params['waypoints']


    """ Create compensation details """
    # setting variable
    reward = 0
    vehicle_length = 0 #값 설정 wheel_base / 2
    vehicle_width = 0  #값 설정 width / 2
    lfp = 20    # basic look forward point
    lfd = 0     # look forward distance

    prev_point = waypoints[closest_waypoint[0]]
    next_point = waypoints[closest_waypoint[1]]
    is_left = False
    is_right = False

    # set look_forward_distance
    for number in range(0, 6):
        point1 = waypoints[prev_point + lfp]
        point2 = waypoints[prev_point + ((lfp) / 2)]

        numerator = abs((point1[0] - prev_point[0] * (prev_point[1] - point2[1]) - (prev_point[0] - point2[0]) * (point1[1] - prev_point[1])))
        denominator = math.sqrt((point1[0] - prev_point[0])**2 + (point1[1] - prev_point[1])**2)
        width = numerator / denominator

        if width > ((track_width / 2) + (vehicle_width / 2)):
            lfp -= 1
            continue
        else:
            lfd += math.sqrt((point1[0] - current_pose_x) ** 2 + (point1[1] - current_pose_y) ** 2)
            break

    # reward about steering angle with pure pursuit
    point = waypoints[prev_point + lfp]
    target_x = point[0] - current_pose_x
    target_y = point[1] - current_pose_y

    slope = target_y / target_x
    theta_radians = math.atan(slope)
    theta_degrees = math.degrees(theta_radians)

    if target_x > 0 and target_y >= 0:
        theta = theta_degrees
    elif target_x < 0 and target_y >= 0:
        theta = 180.0 + theta_degrees
    elif target_x < 0 and target_y < 0:
        theta = -180.0 + theta_degrees
    elif target_x > 0 and target_y < 0:
        theta = -theta_degrees

    if heading >= theta:
        alpha = abs(heading - theta)
        is_right = not is_right
    elif heading < theta:
        alpha = abs(theta - heading)
        is_left = not is_left
    
    if is_left == True:
        target_steering = math.atan(vehicle_length * 2 * math.sin(alpha) / lfd)
    elif is_right == True:
        target_steering = math.atan(vehicle_length * 2 * math.sin(alpha) / lfd)
        target_steering = -target_steering
    direction_diff = abs(target_steering - steering_angle)
    reward_steering = math.cos(direction_diff**3)

    # steps

    # reward about progress
    global progress_prev
    if steps <= 2:
        progress_prev = progress
    
    delta_progress = progress - progress_prev
    delta_projection = track_length * (delta_progress / 100)
    delta_projection = max(min(delta_projection, 1), -1)

    reward_progress = delta_projection ** 3
    if progress == 100:
        reward_progress += 100
    progress_prev = progress


    """ Reward weight sum """
    reward = reward_steering + reward_progress
    return reward