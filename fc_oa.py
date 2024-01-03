 
from doctest import run_docstring_examples
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
#(Open src terminal inside turtlebot) -> python3 fc_oa_sim.py

global rules # we make rules global so functions can access them
rules = {"n_n":["l","s"],
         "n_m":["r","s"],
         "n_f":["r","s"],
         "m_n":["l","s"],
         "m_m":["l","s"],
         "m_f":["r","m"],
         "f_n":["l","s"],
         "f_m":["l","m"],
         "f_f":["f","m"]
        }

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

############################################### Fuzzy Functions ###############################################
def eq_of_line(x, point1, point2): 
    """ Function to calculate the equation of a line and return the 'y' value (standard formula) """
    x1, y1 = point1
    x2, y2 = point2
    m = (y2 - y1) / (x2 - x1) # slope
    b = y1 - m * x1 # y intercept
    y = m * x + b
    return y

### input trapezoid functions ###
def near_func(x, first_p, second_p):
    """ left shoulder trapezoid function """
    if x <= first_p:
        y = 1
    elif first_p < x < second_p: # falling slope section of the functon
        y = eq_of_line(x,(first_p,1),(second_p,0)) # when the input is located in the slope
    elif x >= second_p:
        y = 0
    return y

def middle_func(x, first_p, second_p, third_p, fourth_p):
    """ middle trapezoid functon """
    if second_p <= x <= third_p: # top of the trapezoid or triangle
        y = 1
    elif first_p < x < second_p: # rising slope section of the functon
        y = eq_of_line(x,(first_p,0),(second_p,1))
    elif third_p < x < fourth_p: # falling slope section of the functon
        y = eq_of_line(x,(third_p,1),(fourth_p,0))
    elif x <= first_p or x >= fourth_p:
        y = 0
    return y

def far_func(x, third_p, fourth_p):
    """ right shoulder trapezoid function """
    if x <= third_p:
        y = 0
    elif third_p < x < fourth_p: # rising slope section of the functon
        y = eq_of_line(x,(third_p,0),(fourth_p,1))
    elif x >= fourth_p:
        y = 1
    return y
### end of trapezoid functions ###

def fuzzify(x, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4):
    """ fuzzyfication process for a single input, send the current input vale to each trapezoid function along with the established function points.
     Builds a dictionary which holds the near, middle, and far fuzzified inputs ex,  {'n': 0.5999999999999996, 'm': 0.40000000000000036} """
    fuzzy = {"n":near_func(x, m1_p1, m1_p2), "m":middle_func(x, m2_p1, m2_p2, m2_p3, m2_p4), "f":far_func(x, m3_p3, m3_p4)}
    filtered_fuzzy = {key: value for key, value in fuzzy.items() if value != 0} # Removing values that are == 0
    return filtered_fuzzy

def get_combinations(dic1, dic2, operation="min"):
    """ function to generate a dcitionary list of all teh combinations of the fuzzy values from the inputs. Default is min
    but can easily bt switched to max operation"""
    combination_dict = {}

    for key_rfs in dic1:
        for key_rbs in dic2:
            combination_key = f"{key_rfs}_{key_rbs}"
            value_rfs = dic1[key_rfs]
            value_rbs = dic2[key_rbs]
            if operation == "min":
                comparison_result = min(value_rfs, value_rbs)
            else: #max
                comparison_result = max(value_rfs, value_rbs)
            combination_dict[combination_key] = comparison_result

    return combination_dict


def angular_func(x):
    """ returns the center of gravity of the angular triangular output function """
    if x == "r": # right
        memb = [-0.4, -0.3, -0.2]
        return sum(memb) / len(memb)
    elif x == "f": # forward
        memb = [-0.2, 0, 0.2]
        return sum(memb) / len(memb)
    else: # left
        memb = [0.2, 0.3, 0.4]
        return sum(memb) / len(memb)

def motor_func(x):
    """ returns the center of gravity of the triangular output function """
    if x == "s": # slow
        memb = [0.04, 0.08, 0.12]
        return sum(memb) / len(memb)
    elif x == "m": # medium
        memb = [0.12, 0.16, 0.20]
        return sum(memb) / len(memb)
    else: # fast
        memb =  [0.20, 0.24, 0.28]
        return sum(memb) / len(memb)
    
def defuzz(combinations, rules):
    """ defuzzification function, makes the final operations to calcualte the angular momentum and motor speed outputs as a list.
    first the angular output is calculated and then the motor;
      the addition of all the combinations * the corresponding center of gravity / addition of all the combinations """
    angular_motor= [0,0]

    for i in [0,1]: # 0 = angular momentum, 1 = motor speed
        numerator = 0
        denominator = 0
        for j in combinations:
            if i == 0: #angular momentum
                numerator += angular_func(rules[j][i]) *  combinations[j]
            else: #motor speed
                numerator += motor_func(rules[j][i]) *  combinations[j]
            denominator += combinations[j] # the denominator is calculated the same for both functions
        angular_motor[i] = numerator / denominator

    return angular_motor

def fuzzy_epoch(rfs, rbs, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4):
    """ putting together all the functions listed above, into a single iteration where this functions recieves the sensors inputs, as well
    as all the points for each input membership function so you can change a value on the fly."""
    fuzzy1 = fuzzify(rfs, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4) # fuzzify the rfs input
    fuzzy2 = fuzzify(rbs, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4)
    combinations = get_combinations(fuzzy1, fuzzy2, operation="min") #calcualte all the combinations for the fuzzified inputs
    defuzzy = defuzz(combinations, rules)
    return defuzzy
############################################### End of Fuzzy Functions ###############################################

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:25]),
        'front2':  find_nearest (msg.ranges[335:360]),
        'bright': find_nearest (msg.ranges[220:250]),
        'fright': find_nearest (msg.ranges[290:320]),
        'fleft':  find_nearest (msg.ranges[40:50]),
        'left':   find_nearest (msg.ranges[85:95])
    }    
    twstmsg_= movement()
   
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    print("------------------------------")
    print("Distance front left: ", regions['front1'])
    print("Distance front right: ", regions['front2'])
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()

    # run through the whole fuzzy proccess, inputing the desired points of each membership function
    defuzzy = fuzzy_epoch(regions['front1'], regions['front2'], m1_p1=0.30, m1_p2=0.45, m2_p1=0.30, m2_p2=0.45, m2_p3=0.45, m2_p4=0.60, m3_p3=0.45, m3_p4=0.60)
    print("Fuzzy Angular", defuzzy[0])
    print("Fuzzy Motor", defuzzy[1])
    print("------------------------------")
    msg.linear.x = defuzzy[1] # motor speed
    msg.angular.z = defuzzy[0] # angular momentum
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
