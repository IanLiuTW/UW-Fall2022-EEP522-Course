#!/usr/bin/env python

import math
import sys

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan

import utils

SCAN_TOPIC = '/car/scan'  # The topic to subscribe to for laser scans
# The topic to publish controls to
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
POSE_TOPIC = '/car/car_pose'  # The topic to subscribe to for current pose of the car
# NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
# The topic to publish to for vizualizing
VIZ_TOPIC = '/laser_wanderer/rollouts'
# the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000  # The penalty to apply when a configuration in a rollout
# goes beyond the corresponding laser scan


'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''


class LaserWanderer:

    '''
    Initializes the LaserWanderer
      rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
                containing T poses. For each trajectory, the t-th element represents
                the [x,y,theta] pose of the car at time t+1
      deltas: An N dimensional array containing the possible steering angles. The n-th
              element of this array is the steering angle that would result in the 
              n-th trajectory in rollouts
      speed: The speed at which the car should travel
      compute_time: The amount of time (in seconds) we can spend computing the cost
      laser_offset: How much to shorten the laser measurements
    '''

    def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
        # Store the params for later
        self.rollouts = rollouts
        self.deltas = deltas
        self.speed = speed
        self.compute_time = compute_time
        self.laser_offset = laser_offset

        # YOUR CODE HERE
        # Create a publisher for sending controls
        self.cmd_pub = rospy.Publisher(
            CMD_TOPIC, AckermannDriveStamped, queue_size=10)
        # Create a subscriber to laser scans that uses the self.wander_cb callback
        self.laser_sub = rospy.Subscriber(
            SCAN_TOPIC, LaserScan, self.wander_cb)
        # Create a publisher for vizualizing trajectories. Will publish PoseArrays
        self.viz_pub = rospy.Publisher(
            VIZ_TOPIC, PoseArray, queue_size=10)
        self.viz_sub = rospy.Subscriber(
            POSE_TOPIC, PoseStamped, self.viz_sub_cb)  # Create a subscriber to the current position of the car
        # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?

    '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''

    def viz_sub_cb(self, msg):
        # Create the PoseArray to publish. Will contain N poses, where the n-th pose
        # represents the last pose in the n-th trajectory
        pa = PoseArray()
        pa.header.frame_id = '/map'
        pa.header.stamp = rospy.Time.now()

        # Transform the last pose of each trajectory to be w.r.t the world and insert into
        # the pose array
        # YOUR CODE HERE
        N, T, _ = self.rollouts.shape
        yaw = utils.quaternion_to_angle(msg.pose.orientation)
        rot_mat = utils.rotation_matrix(yaw)

        for i in range(N):
            x, y, theta = self.rollouts[i, -1, :]
            rot_x, rot_y = rot_mat.dot(np.array([[x], [y]]))

            pose = Pose()
            pose.position.x = rot_x + msg.pose.position.x
            pose.position.y = rot_y + msg.pose.position.y
            pose.orientation = utils.angle_to_quaternion(theta + yaw)
            pa.poses.append(pose)

        self.viz_pub.publish(pa)

    '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''

    def compute_cost(self, delta, rollout_pose, laser_msg):

        # Initialize the cost to be the magnitude of delta
        # Consider the line that goes from the robot to the rollout pose
        # Compute the angle of this line with respect to the robot's x axis
        # Find the laser ray that corresponds to this angle
        # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose
        # is greater than the laser ray measurement - np.abs(self.laser_offset)
        # Return the resulting cost
        # Things to think about:
        #   What if the angle of the pose is less (or greater) than the angle of the
        #   minimum (or maximum) laser scan angle
        #   What if the corresponding laser measurement is NAN or 0?
        # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION

        # YOUR CODE HERE
        pred_dis = math.sqrt(rollout_pose[0]**2 + rollout_pose[1]**2)
        laser_ranges = len(laser_msg.ranges)
        theta = math.atan2(rollout_pose[1], rollout_pose[0]) * 180 / math.pi
        beam_index = int(laser_ranges//2 + theta*(laser_ranges//360))

        selected_range = 4
        left = max(0, beam_index - selected_range)
        right = min(laser_ranges, beam_index + selected_range)
        cnt, obs_dis = 0, 0
        for i in range(left, right):
            if math.isnan(laser_msg.ranges[i]):
                continue
            cnt += 1
            obs_dis += laser_msg.ranges[i]
        obs_dis = obs_dis / cnt - self.laser_offset if cnt else float('inf')

        cost = np.abs(delta) if pred_dis < obs_dis else MAX_PENALTY
        return cost

    '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''

    def wander_cb(self, msg):
        start = rospy.Time.now().to_sec()  # Get the time at which this function started

        # A N dimensional matrix that should be populated with the costs of each
        # trajectory up to time t <= T
        delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float)
        traj_depth = 0

        # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
        # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
        # as appropriate

        # Pseudo code
        # while(you haven't run out of time AND traj_depth < T):
        #   for each trajectory n:
        #       delta_costs[n] += cost of the t=traj_depth step of trajectory n
        #   traj_depth += 1
        # YOUR CODE HERE
        N, T, _ = self.rollouts.shape

        while (rospy.Time.now().to_sec() - start < self.compute_time and traj_depth < T):
            for n in range(N):
                delta_costs[n] += self.compute_cost(
                    self.deltas[n], self.rollouts[n, traj_depth], msg)
            traj_depth += 1

        # Find the delta that has the smallest cost and execute it by publishing
        # YOUR CODE HERE
        idx = 0
        mid, sum_fh, sum_sh = N//2, 0, 0
        for n in range(N):
            # print(delta_costs[n], self.deltas[n])
            if n < mid:
                sum_fh += delta_costs[n]
            else:
                sum_sh += delta_costs[n]
            if delta_costs[n] < delta_costs[idx]:
                idx = n
        delta = self.deltas[idx]
        correction, gain = 0.04, 1.4  # TODO need to test
        delta += correction if sum_fh > sum_sh else -correction
        delta *= gain

        cmd_msg = AckermannDriveStamped()
        cmd_msg.header.frame_id = '/map'
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.drive.steering_angle = delta
        cmd_msg.drive.speed = self.speed

        self.cmd_pub.publish(cmd_msg)


'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''


def kinematic_model_step(pose, control, car_length):
    # Apply the kinematic model
    # Make sure your resulting theta is between 0 and 2*pi
    # Consider the case where delta == 0.0

    # YOUR CODE HERE
    x, y, theta = pose
    v, delta, dt = control

    dx = np.cos(theta)*v*dt
    dy = np.sin(theta)*v*dt
    dtheta = (v/car_length) * np.tan(delta) * dt
    xnew = x + dx
    ynew = y + dy
    thetanew = theta + dtheta
    thetanew = np.mod(thetanew, 2.0*np.pi)

    return np.array([xnew, ynew, thetanew], dtype=np.float)


'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''


def generate_rollout(init_pose, controls, car_length):
    # YOUR CODE HERE
    T = controls.shape[0]

    rollout = np.zeros((T, 3), dtype=np.float)
    for i in range(T):
        init_pose = kinematic_model_step(init_pose, controls[i], car_length)
        rollout[i] = init_pose
    return rollout


'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''


def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

    deltas = np.arange(min_delta, max_delta, delta_incr)
    N = deltas.shape[0]

    init_pose = np.array([0.0, 0.0, 0.0], dtype=np.float)

    rollouts = np.zeros((N, T, 3), dtype=np.float)
    for i in xrange(N):
        controls = np.zeros((T, 3), dtype=np.float)
        controls[:, 0] = speed
        controls[:, 1] = deltas[i]
        controls[:, 2] = dt
        rollouts[i, :, :] = generate_rollout(init_pose, controls, car_length)

    return rollouts, deltas


def main():

    rospy.init_node('laser_wanderer', anonymous=True)

    # Load these parameters from launch file
    # We provide suggested starting values of params, but you should
    # tune them to get the best performance for your system
    # Look at constructor of LaserWanderer class for description of each var
    # 'Default' values are ones that probably don't need to be changed (but you could for fun)
    # 'Starting' values are ones you should consider tuning for your system
    # YOUR CODE HERE
    speed = 1.0  # Default val: 1.0
    min_delta = -0.34  # Default val: -0.34
    max_delta = 0.341  # Default val: 0.341
    # Starting val: 0.34/3 (consider changing the denominator)
    delta_incr = 0.34/3
    dt = 0.01  # Default val: 0.01
    T = 300  # Starting val: 300
    compute_time = 0.09  # Default val: 0.09
    laser_offset = 1.0  # Starting val: 1.0

    speed = rospy.get_param("speed")
    min_delta = rospy.get_param("min_delta")
    max_delta = rospy.get_param("max_delta")
    delta_incr = rospy.get_param("delta_incr")
    dt = rospy.get_param("dt")
    T = rospy.get_param("T")
    compute_time = rospy.get_param("compute_time")
    laser_offset = rospy.get_param("laser_offset")

    # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
    car_length = rospy.get_param("/car/vesc/chassis_length", 0.33)

    # Generate the rollouts
    rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                             delta_incr, dt, T, car_length)

    # Create the LaserWanderer
    lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)

    # Keep the node alive
    rospy.spin()


if __name__ == '__main__':
    main()
