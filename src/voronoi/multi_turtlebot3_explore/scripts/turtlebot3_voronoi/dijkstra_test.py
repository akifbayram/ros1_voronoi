#! /usr/bin/env python

import rospy
import rospkg
from rospy.timer import sleep
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from math import cos, radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

import threading
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState 

from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
#from kobuki_msgs.msg import BumperEvent
import time

import tensorflow
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input
# from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam, get
import keras.backend as K
import gym
import numpy as np
import math
import random
import threading
from std_srvs.srv import Empty

from nav_msgs.msg import OccupancyGrid

import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  



class InfoGetter(object):
    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._msg = None

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._msg

class GameState:

    def __init__(self):
        self.talker_node = rospy.init_node('voronoi', anonymous=False)
        self.pose_ig = InfoGetter()
        self.collision_ig = InfoGetter()
        self.lambda_value = 1
        self.move_cmd = Twist()

        self.point_pub = rospy.Publisher('point_topic', Point, queue_size=10)

        # tf
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)
        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'

        self.robot_name = ['tb3_0','tb3_1','tb3_2']             # enter all for multiple robots
        self.position = {'tb3_0':Point(),'tb3_1':Point(),'tb3_2':Point()}

        self.rotation = {'tb3_0':0.0,'tb3_1':0.0,'tb3_2':0.0}
        
        # record target position
        self.record_info_node = {'tb3_0':[],'tb3_1':[],'tb3_2':[]}

        # set netx target position
        self.next_target_node = {'tb3_0':[],'tb3_1':[],'tb3_2':[]}

        # whether arrive information node 
        self.arr_info_node = {'tb3_0':False,'tb3_1':False,'tb3_2':False}

        # status of goal reach thread
        self.thread_is_alive = {'tb3_0':False,'tb3_1':False,'tb3_2':False}

        # whether complete
        self.done = False

        # each robot share its own position max distacen
        self.communication_max_range = 6

        # Is there any information node point within the detection range
        self.detect_info_node = False

        # the value of laser_crashed_value is True when a collision is imminent,else False
        self.laser_crashed_value = {'tb3_0':False,'tb3_1':False,'tb3_2':False}
        
        self.map_merge_data = OccupancyGrid()
        self.map1_free_num = 6251.0
        self.map3_free_num = 9000.0
        self.target_explored_region_rate = 0.8
        self.done = False

        self.rate = rospy.Rate(100) # 10hz

        # crush default value
        self.crash_indicator = 0

        # observation_space and action_space
        self.state_num = 28 #685                 # when you change this value, remember to change the reset default function as well
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)


        self.laser_reward = 0

        # set turtlebot index in gazebo world
        self.model_index = 10 #25

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
    
    # this gets the next target nodes for all the robots after the reset
    def get_init_info_node(self):
        for name in self.robot_name:
            laser_ig = InfoGetter()
            rospy.Subscriber(name+'/scan', LaserScan, laser_ig)
            laser_msg = laser_ig.get_msg()
            self.laser_msg_range_max = laser_msg.range_max
            laser_values = laser_msg.ranges
            option_target_point = []
            for j in range(len(laser_values)):
                if laser_values[j] == np.inf:
                    theta = j*(laser_msg.angle_increment) + (math.pi/2 - laser_msg.angle_max)
                    option_target_point_x = self.position[name].x + (self.laser_msg_range_max * math.sin(theta) )
                    option_target_point_y = self.position[name].y - (self.laser_msg_range_max * math.cos(theta) )
                    option_target_point.append([option_target_point_x,option_target_point_y])
            option_target_point = self.voronoi_select_point(name,option_target_point)
            self.next_target_node[name] = random.choice(option_target_point)
            self.record_info_node[name].append(self.next_target_node[name])

    def reset(self):
        self.laser_crashed_value = {'tb3_0':False,'tb3_1':False,'tb3_2':False}
        self.rate.sleep()
        self.crash_indicator = 0

        # location initialization
        state_msg_0 = ModelState()    
        state_msg_0.model_name = 'tb3_0'
        state_msg_0.pose.position.x = -3  # -3 ,-4, -5.2,
        state_msg_0.pose.position.y = -1.5 #random_turtlebot_y
        state_msg_0.pose.position.z = 0
        state_msg_0.pose.orientation.x = 0
        state_msg_0.pose.orientation.y = 0
        state_msg_0.pose.orientation.z = 0
        state_msg_0.pose.orientation.w = 0

        state_msg_1 = ModelState()    
        state_msg_1.model_name = 'tb3_1'
        state_msg_1.pose.position.x = -3
        state_msg_1.pose.position.y = 0.0 #random_turtlebot_y
        state_msg_1.pose.position.z = 0
        state_msg_1.pose.orientation.x = 0
        state_msg_1.pose.orientation.y = 0
        state_msg_1.pose.orientation.z = 0
        state_msg_1.pose.orientation.w = 0

        state_msg_2 = ModelState()    
        state_msg_2.model_name = 'tb3_2'
        state_msg_2.pose.position.x = -3
        state_msg_2.pose.position.y = 1.5 #random_turtlebot_y
        state_msg_2.pose.position.z = 0
        state_msg_2.pose.orientation.x = 0
        state_msg_2.pose.orientation.y = 0
        state_msg_2.pose.orientation.z = 0
        state_msg_2.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/set_model_state')

        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg_0)
            set_state(state_msg_1)
            set_state(state_msg_2)

            #resp_target = set_state(state_target_msg)

        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

        # get position of robot
        for name in self.robot_name:
            self.set_tf(name)
            time.sleep(0.1)
            self.position[name],self.rotation[name] = self.get_odom(name)


        for name in self.robot_name:
            pub = rospy.Publisher(name+'/cmd_vel', Twist, queue_size=1)
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            self.rate.sleep()
            pub.publish(self.move_cmd)
            self.rate.sleep()
            

        # get target node
        self.get_init_info_node()
        for name in self.robot_name:
            print(name,":target position",self.next_target_node[name][0],self.next_target_node[name][1])

        initial_state = np.ones(self.state_num)
        #initial_state[self.state_num-2] = 0
        initial_state[self.state_num-1] = 0
        initial_state[self.state_num-2] = 0
        initial_state[self.state_num-3] = 0
        initial_state[self.state_num-4] = 0

        self.rate.sleep()
        initial_state = [initial_state]*3
        return initial_state

    def set_tf(self,robot_name):
        try:
            self.tf_listener.waitForTransform(robot_name+self.odom_frame, robot_name+'/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = robot_name + '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(robot_name+ self.odom_frame, robot_name+'/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = robot_name + '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
    
    def get_odom(self,robot_name):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(robot_name+ self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans), rotation[2]
    
    def line_distance(self,position0_x,position0_y,position1_x,position1_y):
        return math.sqrt((position0_x - position1_x)**2 + (position0_y - position1_y)**2)

    
    def whether_detect_info_node(self):
        pass

    # the number of robot 
    def num_robot_site(self,robot_name):
        option_site = []
        for name in self.robot_name:
            if robot_name == name:
                pass
            else:
                if self.line_distance(self.position[robot_name].x,self.position[robot_name].y,self.position[name].x,self.position[name].y) <=6:
                    option_site.append(name)
        #print(option_site)
        return option_site
            
    # select new option_target_point through voronoi algorithm
    # selects the option target points which have higher distance than all its neighbouriing robots  (NOT COMMON!!!!!)
    def voronoi_select_point(self,robot_name,option_target_point):
        option_site = self.num_robot_site(robot_name)  # this function creates this option_site list with neighbour robots excluding it
        voronoi_option_target_point = []
        if len(option_site)==0:   # no neighbouring robots
            return option_target_point
        for i in range(len(option_target_point)):
            j=0
            for name in option_site:   # for every neighbouring robots
                distance = self.line_distance(self.position[name].x,self.position[name].y,option_target_point[i][0],option_target_point[i][1])
                # this is the distance between the current option target point and the current robot in the option site
                if distance > self.laser_msg_range_max:
                    j+=1
                if distance < self.laser_msg_range_max:
                    pass
                if j== len(option_site): # this is when all the robots in the option site has a distance, highier than that option target point
                    voronoi_option_target_point.append(option_target_point[i])
        return voronoi_option_target_point

    def get_min_Omega_distance_point(self,robot_name,option_target_point):
        Omega = 0 # distance of d_ik and phi_ik
        min_Omega = np.inf
        for i in range(len(option_target_point)):
            Omega = self.lambda_value*(self.line_distance(self.record_info_node[robot_name][0][0],self.record_info_node[robot_name][0][1],option_target_point[i][0],option_target_point[i][1])) \
                     + (1-self.lambda_value)*(self.line_distance(self.position[robot_name].x, self.position[robot_name].y,option_target_point[i][0],option_target_point[i][1]))
            if Omega < min_Omega:
                min_Omega = Omega
                return option_target_point[i]
    
    # filters out the laser obtained frontier points using voronoi algorithm and then selects the best next target point using min omega distance!!!
    def get_record_next_info_node(self,robot_name,option_target_point):
        if(self.arr_info_node[robot_name] == True):  # the distance between the robot and the next target point is within the threshold
            option_target_point = self.voronoi_select_point(robot_name,option_target_point) # Further select the next point through the Voronoi algorithm
            if len(option_target_point) == 0:
                return False
            else:
                self.next_target_node[robot_name] = self.get_min_Omega_distance_point(robot_name,option_target_point)
                print(robot_name,":target position",self.next_target_node[robot_name][0],self.next_target_node[robot_name][1])
                self.record_info_node[robot_name].append(self.next_target_node[robot_name])
                self.arr_info_node[robot_name] = False
                return True

    def map_data_handle(self):
        start_time = time.time()
        while True:
            map_ig = InfoGetter()
            rospy.Subscriber('/map_merge/map',OccupancyGrid,map_ig)
            map_msg = map_ig.get_msg()
            self.map_merge_data.data = map_msg.data

            free_num=0
            explored_region_rate =0.0
            for data in self.map_merge_data.data:
                if data == 0:
                    free_num += 1
            explored_region_rate = free_num/self.map1_free_num
            print(explored_region_rate)
            time.sleep(1)
            if explored_region_rate >= self.target_explored_region_rate:
                self.done = True
            if explored_region_rate > 0.9:
                end_time = time.time()
                print("Time taken for the exploration is %f when lambda is %f" % (end_time - start_time, self.lambda_value))

    def game_step(self, robot_name):
        if self.thread_is_alive[robot_name] == True:
            return
        # pub = rospy.Publisher(robot_name+'/cmd_vel', Twist, queue_size=1)
        move_base = actionlib.SimpleActionClient( robot_name + '/move_base', MoveBaseAction)  

        # Maximum number of retries
        max_retries = 10
        retry_count = 0

        # Wait for the server with a retry mechanism
        while not move_base.wait_for_server(rospy.Duration(5.0)):
            retry_count += 1
            rospy.loginfo("Waiting for the move_base server...")
            if retry_count >= max_retries:
                rospy.logerr("move_base server is not available after multiple attempts.")
                # Handle the failure (exit, raise exception, etc.)
                exit()
                rospy.signal_shutdown("move_base server unavailable")
                break
        if retry_count < max_retries:
            rospy.loginfo("Connected to move_base server.")
            # Proceed with your move_base actions

        laser_ig = InfoGetter()
        rospy.Subscriber(robot_name+'/scan', LaserScan, laser_ig)

        start_time = time.time()
        record_time = start_time

        

        self.base_frame = robot_name + '/base_footprint'

        self.position[robot_name], self.rotation[robot_name]= self.get_odom(robot_name)  # gets the Odometry the robot
        turtlebot_x_previous = self.position[robot_name].x
        turtlebot_y_previous = self.position[robot_name].y

        if self.line_distance(self.position[robot_name].x,self.position[robot_name].y,self.next_target_node[robot_name][0],self.next_target_node[robot_name][1])<0.5:
            # this checks whether the cartesian distance between the current position and the next target node
            self.arr_info_node[robot_name] = True
        else:
            self.arr_info_node[robot_name] = False

        
        # get list of optional target point using lidar scans
        #self.arr_info_node[robot_name] = True
        option_target_point = []
        theta= 0
        if self.arr_info_node[robot_name] == True:
            laser_msg = laser_ig.get_msg() # gets the laser msg
            laser_values = laser_msg.ranges
            for i in range(len(laser_values)):
                if laser_values[i] == np.inf:
                    theta = (i+1)*laser_msg.angle_increment
                    option_target_point_x = turtlebot_x_previous + (self.laser_msg_range_max * math.sin(theta))
                    option_target_point_y = turtlebot_y_previous - (self.laser_msg_range_max * math.cos(theta))
                    option_target_point.append([option_target_point_x,option_target_point_y])
            if len(option_target_point):
                self.get_record_next_info_node(robot_name,option_target_point)
            else:
                self.rate.sleep()
                self.laser_crashed_value[robot_name] =1 
                # prepare the normalized laser value and check if it is crash
        
        laser_msg = laser_ig.get_msg()
        laser_values = laser_msg.ranges
        for i in range(len(laser_values)):
            if (laser_values[i] < 0.3):
                self.laser_crashed_value[robot_name] = True
                #self.reset()
            if (laser_values[i] < 0.2):
                self.laser_crashed_value[robot_name] = True
                self.reset()
                break
        target_x = self.next_target_node[robot_name][0]
        target_y = self.next_target_node[robot_name][1]


        move_base.wait_for_server(rospy.Duration(15.0))
        
        translation, rotation = self.get_odom(robot_name)
        angle_to_goal=math.atan2(target_y-translation.y,  target_x-translation.x)
        q_angle = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes='sxyz')
        q = Quaternion(*q_angle)

        # target = Pose(Point(target_x, target_y, 0.000), Quaternion(0.000, 0.000, 0.001, 0.000))  
        target = Pose(Point(target_x, target_y, 0.000), q) 
        goal = MoveBaseGoal()  
        goal.target_pose.pose = target  
        goal.target_pose.header.frame_id = 'map' 
        goal.target_pose.header.stamp = rospy.Time.now()

        # self.point_pub.publish(goal)

        # Start the goal in a separate thread
        thread = "thread_" + robot_name
        thread = threading.Thread(target=self.send_goal_async, args=(move_base, goal,robot_name))
        thread.start()
        # rospy.loginfo("Setting a new goal to : " + str(robot_name))
        self.thread_is_alive[robot_name] = True

    def send_goal_async(self, move_base, goal, robot_name):
        # Send the goal
        move_base.send_goal(goal)

        # Monitor the goal asynchronously
        finished_within_time = move_base.wait_for_result(rospy.Duration(100))
        self.thread_is_alive[robot_name] = False
        if not finished_within_time:  
            move_base.cancel_goal() 
            rospy.loginfo("Timed out achieving goal")  
        else:  
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo(robot_name  + " : Goal succeeded!")
            else:  
                rospy.loginfo(robot_name+ "Goal failed with state: %d" % state)

if __name__ == '__main__':
    try:
        sess = tensorflow.compat.v1.Session()    # Change due to new tensorflow version
        K.set_session(sess)

        game_state = GameState()
        rospy.loginfo("Gamestate initialised!!!")
        game_state.reset()
        rospy.loginfo("Gamestate reset!!!")

        thread0 = threading.Thread(target=game_state.map_data_handle)
        thread0.daemon = True  # Set as daemon thread
        thread0.start()

        trial_len  = 10
        while True:
            for k in range(3):
                game_state.game_step(game_state.robot_name[k])

    except rospy.ROSInterruptException:
        pass
    