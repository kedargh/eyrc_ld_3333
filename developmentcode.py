#!/usr/bin/env python3

"""
Controller for the drone
"""

# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool



MIN_ROLL = 1350
BASE_ROLL = 1500
MAX_ROLL = 1550
SUM_ERROR_ROLL_LIMIT = 10000
MIN_PITCH = 1350
BASE_PITCH = 1500
MAX_PITCH = 1550
MIN_THROTTLE = 1000
MAX_THROTTLE = 1700

DRONE_WHYCON_POSE = [[0], [0], [0]]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

class DroneController():
    def __init__(self,node):
        self.node= node
        
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"

        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
        self.set_points = [0, 0, 15]         # Setpoints for x, y, z respectively      


        self.error              = [0,0,0]
        self.prev_error         = [0,0,0] 
        self.differential_error = [0,0,0]
        self.sum_error          = [0,0,0]
 

        self.Kp = [1.95    , 2.49   , 2.5 ]#previous values - 42.48 , 53.7 , 25.2
        self.Ki = [63*0.0001 , 63*0.0001  , 0.0401]#previous values - 0.004 , 0.004 , 0.086
        self.Kd = [69.3   , 72.2  , 168.1 ]#previous values - 600 , 600 , 500 



        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_roll"    ,self.pid_tune_roll_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_pitch"   ,self.pid_tune_pitch_callback,1)

        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)        


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    def pid_tune_roll_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1

    def pid_tune_pitch_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

    def check(self, cal_value, max_value, min_value):
        if cal_value > max_value:
            return max_value
        elif cal_value < min_value:
            return min_value
        else:
            return cal_value
        
    def pid(self):

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        try:
            '''  self.error[0] = (self.drone_whycon_pose_array.poses[0].position.x - self.set_points[0])
            self.error[1] = self.drone_whycon_pose_array.poses[0].position.y - self.set_points[1]
            self.error[2] = self.drone_whycon_pose_array.poses[0].position.z - self.set_points[2]
            print(self.error)


            self.differential_error[0] = self.error[0] - self.prev_error[0]
            self.differential_error[1] = self.error[1] - self.prev_error[1]
            self.differential_error[2] = self.error[2] - self.prev_error[2]
            print(self.differential_error)

            self.sum_error[0] = self.sum_error[0] + self.error[0]
            self.sum_error[1] = self.sum_error[1] + self.error[1]
            self.sum_error[2] = self.sum_error[2] + self.error[2]
            print(self.sum_error)

            self.rc_message.rc_roll = int(1500 + self.error[0]*self.Kp[0] + self.differential_error[0]*self.Kd[0]+ self.sum_error[0]*self.Ki[0])
            self.rc_message.rc_roll = self.check(self.rc_message.rc_roll , MAX_ROLL , MIN_ROLL)
            print("roll ",self.rc_message.rc_roll)

            self.rc_message.rc_pitch  = (int(1560 + self.error[1]*self.Kp[1] + self.differential_error[1]*self.Kd[1] + self.sum_error[1]*self.Ki[1])*1)
            self.rc_message.rc_pitch = self.check(self.rc_message.rc_pitch , MAX_PITCH, MIN_PITCH)
            print("Pitch ",self.rc_message.rc_pitch)

            print("kp is ",self.error[2]*self.Kp[2],"kd is ",self.differential_error[2]*self.Kd[2],"ki is ",self.sum_error[2]*self.Ki[2])
            self.rc_message.rc_throttle = int(1200 + self.error[2]*self.Kp[2] + self.differential_error[2]*self.Kd[2] + self.sum_error[2]*self.Ki[2])
            print("throttle value is ",self.rc_message.rc_throttle)
            self.rc_message.rc_throttle = self.check(self.rc_message.rc_throttle, MAX_THROTTLE, MIN_THROTTLE)
            print("Throttle ",self.rc_message.rc_throttle)

            self.prev_error[0]=self.error[0]
            self.prev_error[1]=self.error[1]
            self.prev_error[2]=self.error[2]'''
            self.error[0] = round(-(self.drone_whycon_pose_array.poses[0].position.x - self.set_points[0]),2)
            self.error[1] = round(self.drone_whycon_pose_array.poses[0].position.y - self.set_points[1],2)
            self.error[2] = round(self.drone_whycon_pose_array.poses[0].position.z - self.set_points[2],2)
            print(self.error)
            self.differential_error[0] = self.error[0] - self.prev_error[0]
            self.differential_error[1] = self.error[1] - self.prev_error[1]
            self.differential_error[2] = self.error[2] - self.prev_error[2]

            self.sum_error[0] = self.sum_error[0] + self.error[0]
            self.sum_error[1] = self.sum_error[1] + self.error[1]
            self.sum_error[2] = self.sum_error[2] + self.error[2]

            self.rc_message.rc_roll = int(1500 + self.error[0]*self.Kp[0] + self.differential_error[0]*self.Kd[0]+ self.sum_error[0]*self.Ki[0])
            self.rc_message.rc_roll = self.check(self.rc_message.rc_roll , MAX_ROLL , MIN_ROLL)
            self.rc_message.rc_pitch = int(1500 + self.error[1]*self.Kp[1] + self.differential_error[1]*self.Kd[1] + self.sum_error[1]*self.Ki[1])
            self.rc_message.rc_pitch = self.check(self.rc_message.rc_pitch , MAX_PITCH , MIN_PITCH)
            self.rc_message.rc_throttle = int(1450 + self.error[2]*self.Kp[2] + self.differential_error[2]*self.Kd[2] + self.sum_error[2]*self.Ki[2])
            self.rc_message.rc_throttle = self.check(self.rc_message.rc_throttle , MAX_THROTTLE , MIN_THROTTLE)


        # Similarly calculate error for y and z axes 
        
        except Exception as err:
            print(err)

        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # self.integral[0] = (self.integral[0] + self.error[0])
        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        # Save current error in previous error

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis


        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch

        
    #------------------------------------------------------------------------------------------------------------------------


        #self.publish_data_to_rpi( roll = self.rc_message.rc_roll, pitch = self.rc_message.rc_pitch, throttle = self.rc_message.rc_throttle)
        self.publish_data_to_rpi( self.rc_message.rc_roll, self.rc_message.rc_pitch, self.rc_message.rc_throttle)
        #Replace the roll pitch and throttle values as calculated by PID 
        self.prev_error[0]=self.error[0]
        self.prev_error[1]=self.error[1]
        self.prev_error[2]=self.error[2]
        
        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.0,
                zero_error=0.0,
            )
        )


    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)

        # BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([roll, pitch, throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return
            order = 3
            fs = 60
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                 self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL

        elif self.rc_message.rc_pitch > MAX_PITCH:
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH

        elif self.rc_message.rc_throttle > MAX_THROTTLE:
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE

        # Similarly add bounds for pitch yaw and throttle 

        self.rc_pub.publish(self.rc_message)


    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone 

    def disarm(self):
        #while 1:
        #    self.pid()
        #    if self.set_points > [0,0,24] and ((self.error[0]<0.5 or self.error[0]>-0.5) and (self.error[1]>-0.5 or self.error[1]<0.5) and (self.error[1]>-0.5 or self.error[1]<0.5)):
        #        self.set_points = [0,0,(self.set_points[2]+1)]
        #        print("disarmings",self.set_points)
#
       #
        #    if (self.error[0]<0.5 or self.error[0]>-0.5) and (self.error[1]>-0.5 or self.error[1]<0.5) and (self.error[1]>-0.5 or self.error[1]<0.5):
        #        break
        #    else:
        #        self.pid()

        while self.rc_message.rc_throttle !=500:
                self.rc_message.rc_throttle = self.rc_message.rc_throttle - 5
                print(self.rc_message.rc_throttle)
                self.publish_data_to_rpi( 1500,1500,self.rc_message.rc_throttle)
                time.sleep(2)




def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 1:
                node.get_logger().error("Unable to detect WHYCON poses")
            rclpy.spin_once(node) # Sleep for 1/30 secs        

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
