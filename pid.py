
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

#Steps to launch simulator
#(Open turtlebot folder terminal) export TURTLEBOT3_MODEL=burger -> ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#(Open src terminal inside turtlebot) -> python3 fstop_sim.py

# Initializing the error variables 
integral_total = 0.0
previous_error = 0.0

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:15]),
        'front2':  find_nearest (msg.ranges[345:360]),
        'right':  find_nearest(msg.ranges[265:275]),
        'fright': find_nearest (msg.ranges[310:320]),
        'fleft':  find_nearest (msg.ranges[40:50]),
        'left':   find_nearest (msg.ranges[85:95])
        
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

def PIDcontroller(current_dis_right, KP = 0.8, KI = 0.025, KD = 10, maxmin = 2, des_dis = 0.35):
    """ PID controller function, takes as input the tunning constants and returns the corresponding angular correction so the robot turns properly """
    global integral_total, previous_error # Making these variables global so the error doesn't disappear after every "epoch"

    current_error = current_dis_right - des_dis # current stae - goal state
    proportional = KP * current_error # calcualting proportional
    if proportional >= maxmin: # just in case the error gets to big
        proportional = maxmin
    elif proportional <= -maxmin:
        proportional = -maxmin

    integral_total += current_error # calcualting integral
    if integral_total >= maxmin:  # just in case the error gets to big
        integral_total = maxmin
    elif integral_total <= -maxmin:
        integral_total = -maxmin
    integral = KI * integral_total
    
    derrivative = KD * (current_error - previous_error) # calcualting derrivative
    if derrivative >= maxmin:  # just in case the error gets to big
        derrivative = maxmin
    elif derrivative <= -maxmin:
        derrivative = -maxmin
    previous_error = current_error

    agular_correction = proportional + integral + derrivative

    print("Proportional: ", proportional)
    print("Integral: ", integral)
    print("Derrivative: ", derrivative)

    return -1 * agular_correction # Multiplied by a -1 since output is reversed 

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    print("------------------------------") #Keeping track of the robots readings
    print("Distance in front region: ", regions_['front1'],regions_['front2'])
    print("Distance right: ", regions['right'])
    print("Distance left: ", regions['left'])
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()

    msg.linear.x = 0.1 #postive move forward
    xx = PIDcontroller(regions['right']) #calling the PIDcontroller function and passing along the reading for the right sensor
    print("Angular correction: ", xx)
    print("------------------------------")
    msg.angular.z = xx
    return msg

#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)

def main():
    global pub_, mynode_
    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')
    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)
    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)
    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)
    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
