import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.1
CYCLE_PENALTY = -10


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    if -4.5 > x > -10.0 and 5.0 > y > 3.8:
        goal_ok = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goal_ok = False

    # Cafe bar
    if 0 > x > -2 and 7.5 > y > 2.5:
        goal_ok = False

    # Right-side wall - middle
    if 6 > x > 3.9 and 6.5 > y > -7.0:
        goal_ok = False

    # Fridge
    if -3.5 > x > -5.5 and 1.2 > y > -0.5:
        goal_ok = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goal_ok = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goal_ok = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goal_ok = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goal_ok = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goal_ok = False

    if 10.0 > x > 4.5 and -3.3 > y > -4.2:
        goal_ok = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goal_ok = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goal_ok = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False

    return goal_ok


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "r1"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.2
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.2
        self.set_self_state.pose.orientation.w = 1.0

        # Add for dynamic obstacle detection
        self.previous_lidar_data = None
        self.lidar_scan_time = 0.1
        self.collision_threshold = 0.35
        self.dynamic_collision_threshold = 0.75 # processing + actuation delay = 0.5s and safety margin 0.25m

        # Add for cycle detection
        self.state_history = []
        self.state_history_limit = 50
        self.cycle_threshold = 0.25
        self.cycle_detected = False
        self.exploration_noise_temporary = 1  # Temporary exploration noise to escape cycles
        self.min_exploration_noise = 0.1  # minimum exploration noise
        self.max_exploration_noise = 1
        self.last_position = None

        # Initialize for reward calculation
        self.time_step = 0

        # Subscribe to the odometry topic to get velocity
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_velocity = None

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        port = "11311"
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("/r1/cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        # self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.velodyne = rospy.Subscriber(
            "/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1
        )
        self.odom = rospy.Subscriber(
            "/r1/odom", Odometry, self.odom_callback, queue_size=1
        )

    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, v):
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break

    def odom_callback(self, od_data):
        self.last_odom = od_data

        # Callback function to handle odometry messages
        self.current_velocity = od_data.twist.twist

    # # Perform an action and read a new state
    # def step(self, action):
    #     action = np.array(action)

    #     # # Modify the action with temporary exploration noise if a cycle is detected
    #     # if self.cycle_detected:
    #     #     noise = np.random.randn(*action.shape) * self.exploration_noise_temporary
    #     #     action += noise
    #     #     # print("Exploration noise applied.")

    #     # Execute the action in the environment and observe the next state and reward
    #     next_state, reward, done, info = self._step(action)

    #     # print(f"Checking for cycles with threshold {self.cycle_threshold}")

    #     # # Check for cycles by comparing the next state with the history of states
    #     # cycle_detected = False
    #     # for past_state in self.state_history:
    #     #     if np.linalg.norm(next_state - past_state) < self.cycle_threshold:
    #     #         # print(f"Cycle detected with state difference {np.linalg.norm(next_state - past_state)}")
    #     #         cycle_detected = True
    #     #         reward += CYCLE_PENALTY
    #     #         # print("Cycle detected. Penalty applied.")
    #     #         break

    #     # self.cycle_detected = cycle_detected
        
    #     # # Update the state history, maintaining the history limit
    #     # self.state_history.append(next_state.tolist())
    #     # if len(self.state_history) > self.state_history_limit:
    #     #     self.state_history.pop(0)

    #     # # Adjust exploration noise based on detections
    #     # self.adjust_exploration_noise()

    #     return next_state, reward, done, info

    # def adjust_exploration_noise(self):
    #     if self.cycle_detected:
    #         self.exploration_noise_temporary *= 1.1  # Increase noise
    #     else:
    #         self.exploration_noise_temporary *= 0.9  # Decrease noise
    #     self.exploration_noise_temporary = np.clip(self.exploration_noise_temporary, self.min_exploration_noise, self.max_exploration_noise)

    def step(self, action):
        target = False
        self.time_step += 1

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(self.velodyne_data)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Ensure odometry data is not None before proceeding
        if self.last_odom is None:
            print("Odometry data is not available.")
            # Handle the case where odometry data is not available, e.g., by skipping this step or using default values
            # For this example, let's use default values for odom_x and odom_y, and skip calculations that require last_odom
            self.odom_x = 0
            self.odom_y = 0
            angle = 0  # Default angle value
        else:
            # Calculate robot heading from odometry data
            self.odom_x = self.last_odom.pose.pose.position.x
            self.odom_y = self.last_odom.pose.pose.position.y
            quaternion = Quaternion(
                self.last_odom.pose.pose.orientation.w,
                self.last_odom.pose.pose.orientation.x,
                self.last_odom.pose.pose.orientation.y,
                self.last_odom.pose.pose.orientation.z,
            )
            euler = quaternion.to_euler(degrees=False)
            angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser, self.time_step)
        return state, reward, done, target

    def reset(self):
         # Reset the environment to a starting state
        # self.state_history.clear()  # Clear the state history
        # self.cycle_detected = False  # Reset cycle detection
        self.time_step = 0 # Reset time step counter for the new episode
        start_state = self._reset()  # Implement the reset logic to get the starting state

        # Ensure initial velocities are zero
        self.publish_zero_velocity()
        time.sleep(0.6)  # Give some time for the robot to stabilize

        # start_state = self._reset()

        # Log the current velocity
        if self.current_velocity is not None:
            rospy.loginfo("Current Velocity Linear: %s, Angular: %s", 
                        self.current_velocity.linear, 
                        self.current_velocity.angular)
        else:
            rospy.loginfo("No velocity data available.")

        # Gradually increase speed
        for step in range(1, 6):
            vel_cmd = Twist()
            vel_cmd.linear.x = step * 0.2
            vel_cmd.angular.z = 0
            time.sleep(0.8)
            self.vel_pub.publish(vel_cmd)
        return start_state

    def publish_zero_velocity(self):
        vel_cmd = Twist()  # Zero velocity
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0
        self.vel_pub.publish(vel_cmd)

    def _reset(self):
        # Manually reset the robot's position and orientation ensuring it's not on an obstacle
        quaternion = Quaternion.from_euler(0.0, 0.0, np.random.uniform(-np.pi, np.pi))
        time.sleep(0.5)
        #quaternion = Quaternion.from_euler(0.0, 0.0, 0.0)
        object_state = self.set_self_state

        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.3, 4.3)
            y = np.random.uniform(-9.0, 5.0)
            position_ok = check_pos(x, y)

        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = x
        self.odom_y = y

        # set a random goal in empty space in environment
        self.change_goal()
        self.publish_markers([0.0, 0.0])

        # Return the initial state of the environment
        return self.get_initial_state()

    def get_initial_state(self):
        # Initial lidar data, assuming maximum range for all measurements
        initial_lidar_data = np.ones(self.environment_dim) * 10

        # Calculate initial distance and angle to the goal
        distance_to_goal = np.linalg.norm([self.odom_x - self.goal_x, self.odom_y - self.goal_y])

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0  # Assuming goal direction vector (1, 0) for simplicity
        mag1 = np.sqrt(skew_x**2 + skew_y**2)
        mag2 = 1  # Magnitude of (1, 0) is 1
        beta = np.arccos(dot / (mag1 * mag2))

        angle = np.random.uniform(-np.pi, np.pi)
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Construct the initial state with laser data and robot state ([distance, theta, 0.0, 0.0])
        initial_state = np.concatenate((initial_lidar_data, [distance_to_goal, theta, 0.0, 0.0]))

        return initial_state

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)


    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    # @staticmethod
    # def observe_collision(laser_data):
    #     # Detect a collision from laser data
    #     min_laser = min(laser_data)
    #     if min_laser < COLLISION_DIST:
    #         return True, True, min_laser
    #     return False, False, min_laser

    def observe_collision(self, current_lidar_data):
        if self.previous_lidar_data is None:
            self.previous_lidar_data = current_lidar_data
            return False, False, min(current_lidar_data)

        dynamic_obstacles = []
        for angle in range(len(current_lidar_data)):
            distance_change = current_lidar_data[angle] - self.previous_lidar_data[angle]
            # Detect if there's a significant change in distance that could indicate movement
            if abs(distance_change) > self.movement_threshold(distance_change, self.lidar_scan_time):
                dynamic_obstacles.append((angle, current_lidar_data[angle]))

        # Update previous_lidar_data for the next call
        self.previous_lidar_data = current_lidar_data

        # Process dynamic obstacles to determine if there's a collision risk
        collision_risk = self.process_dynamic_obstacles(dynamic_obstacles)

        min_distance = min(current_lidar_data)
        collision = min_distance < self.collision_threshold  # Static collision check

        return collision, collision_risk, min_distance

    def movement_threshold(self, distance_change, time):
        # Define how you determine significant movement; could be a simple fixed value
        # or more complex logic based on velocity estimation.
        return 0.3  # Example threshold in meters

    def process_dynamic_obstacles(self, dynamic_obstacles):
        # Implement logic to determine if any dynamic obstacle poses a collision risk
        # This could involve checking their distance, estimated velocity, and direction of movement.
        for angle, distance in dynamic_obstacles:
            if distance < self.dynamic_collision_threshold:
                return True  # Example check
        return False

    @staticmethod
    def get_reward(target, collision, action, min_laser, time_step):
        reward = 0.0

        if target:
            reward += 150.0
        if collision:
            reward += -100.0

        reward -= 0.02 * time_step # penalty for each time step to encourage reaching the goal quickly

        r3 = lambda x: 1 - x if x < 1 else 0.0
        reward += action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2

        return reward
