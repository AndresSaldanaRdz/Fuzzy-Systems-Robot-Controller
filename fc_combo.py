
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
#(Open src terminal inside turtlebot) -> python3 fc_combo_sim.py

global rules_ref # we make rules global so functions can access them
global rules_oa

# rules for the right edge following
rules_ref = {"n_n":["l","s"],
         "n_m":["l","m"],
         "n_f":["l","f"],
         "m_n":["r","m"],
         "m_m":["f","m"],
         "m_f":["l","s"],
         "f_n":["r","f"],
         "f_m":["r","m"],
         "f_f":["r","m"]
        }

# rules for obstacle avoidance
rules_oa = {"n_n":["l","s"],
         "n_m":["l","s"],
         "n_f":["l","s"],
         "m_n":["l","s"],
         "m_m":["l","s"],
         "m_f":["l","m"],
         "f_n":["l","s"],
         "f_m":["l","m"],
         "f_f":["f","m"]
        }

mynode_ = None
pub_ = None
regions_ = {
    'front_left': 0,
    'front_right': 0,
    'right_front': 0,
    'right_back': 0,
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
        memb = [-0.5, -0.4, -0.3]
        return sum(memb) / len(memb)
    elif x == "f": # forward
        memb = [-0.3, 0, 0.3]
        return sum(memb) / len(memb)
    else: # left
        memb = [0.3, 0.4, 0.5]
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

def fuzzy_epoch(rfs, rbs, rules, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4):
    """ putting together all the functions listed above, into a single iteration where this functions recieves the sensors inputs, as well
    as all the points for each input membership function so you can change a value on the fly."""
    fuzzy1 = fuzzify(rfs, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4) # fuzzify the rfs input
    fuzzy2 = fuzzify(rbs, m1_p1, m1_p2, m2_p1, m2_p2, m2_p3, m2_p4, m3_p3, m3_p4)
    combinations = get_combinations(fuzzy1, fuzzy2, operation="min") #calcualte all the combinations for the fuzzified inputs
    defuzzy = defuzz(combinations, rules)
    return defuzzy

def subsumption(regions):
    """ subsumption function to combine both fuzzy inputs, sets a range in which the onstacle avoidance is triggered, otherwise it deploys reight edge following"""
    # calculate right edge following and obstacle avoidance; motor speed and angular momentum
    # 1 = motor speed # 0 = angular momentum
    if ((regions['front_left'] + regions['front_right'] )/ 2) <=  0.45:
        oa = fuzzy_epoch(regions['front_left'], regions['front_right'], rules_oa, m1_p1=0.30, m1_p2=0.45, m2_p1=0.30, m2_p2=0.45, m2_p3=0.45, m2_p4=0.60, m3_p3=0.45, m3_p4=0.60)
        angular_momentum = oa[0] 
        motor_speed = oa[1]
    else:
        ref = fuzzy_epoch(regions['right_front'], regions['right_back'], rules_ref, m1_p1=0.20, m1_p2=0.35, m2_p1=0.20, m2_p2=0.35, m2_p3=0.35, m2_p4=0.50, m3_p3=0.35, m3_p4=0.50)
        angular_momentum = ref[0]
        motor_speed = ref[1]

    return angular_momentum, motor_speed

def fuzzy_control(regions):
    """ Context blending function to merge 2 fuzzy functionss, calculates d1 and d2 membership functions to decide on which behavior to prioritize"""
    # 1 = motor speed # 0 = angular momentum
    oa = fuzzy_epoch(regions['front_left'], regions['front_right'], rules_oa, m1_p1=0.30, m1_p2=0.45, m2_p1=0.30, m2_p2=0.45, m2_p3=0.45, m2_p4=0.60, m3_p3=0.45, m3_p4=0.60)
    ref = fuzzy_epoch(regions['right_front'], regions['right_back'], rules_ref, m1_p1=0.20, m1_p2=0.35, m2_p1=0.20, m2_p2=0.35, m2_p3=0.35, m2_p4=0.50, m3_p3=0.35, m3_p4=0.50)

    d1 = min([regions['front_left'], regions['front_right']]) # calculate d1 and d2
    d2 = min([regions['right_front'], regions['right_back']])
    d1_memb_val = near_func(d1, 0.5, 0.9) #near because it's a left trapezoid
    d2_memb_val = far_func(d2, 0.1, 0.7) #far because it's a right trapezoid

    print("Membership OA: ", d1_memb_val)
    print("Membership REF: ", d2_memb_val)
    motor_speed = ( d1_memb_val * oa[1] + d2_memb_val * ref[1]) / (d1_memb_val + d2_memb_val + 0.001) # the actual context blending
    angular_momentum = ( d1_memb_val * oa[0] + d2_memb_val * ref[0]) / (d1_memb_val + d2_memb_val + 0.001)

    return angular_momentum, motor_speed

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
        'front_left':  find_nearest (msg.ranges[0:10]),
        'front_right':  find_nearest (msg.ranges[350:360]),
        'right_front': find_nearest (msg.ranges[290:310]),
        'right_back': find_nearest (msg.ranges[230:250]),
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
    print(f"Distance Front | Left: {round(regions['front_left'],2)}  - Right: {round(regions['front_right'],2)}")
    print(f"Distance Right | Front: {round(regions['right_front'],2)} - Back: {round(regions['right_back'],2)}")
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()

    # - . - Choose between subsumption and context blinding - . - #
    #angular_momentum, motor_speed = subsumption(regions)
    angular_momentum, motor_speed = fuzzy_control(regions)

    print("Fuzzy Angular: ", angular_momentum)
    print("Fuzzy Motor: ", motor_speed)
    print("------------------------------")
    msg.linear.x = motor_speed
    msg.angular.z = angular_momentum
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
