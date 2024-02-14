#!/usr/bin/env python3

from swift_msgs.msg import PIDError, RCMessage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray
from swift_msgs.srv import CommandBool
from sensor_msgs.msg import Image
from pid_msg.msg import PidTune
from rclpy.clock import Clock
from rclpy.node import Node
import scipy.signal
import numpy as np
import imutils
import rclpy
import time
import cv2



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

class DroneController():
    def __init__(self,node):
        self.node= node
        self.rc_message = RCMessage()   
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)   
        self.index = 0
        self.bridge = CvBridge()
        self.error              = [0,0,0]
        self.prev_error         = [0,0,0] 
        self.differential_error = [0,0,0]
        self.sum_error          = [0,0,0]

        self.setpoints = [[1.7, 6.23, 18],[-2.08, 6.20, 18] , [-6.02, 6.15, 18] , [-5.82, 1.90, 18],[-5.67, -1.86 , 18],[-5.88 ,-5.88 , 18],[-2.09 ,-5.68, 18],[1.84 , -5.84 , 18],[5.49, -5.94, 18],[5.60, -1.95, 18],[5.40 ,2.11, 18],[5.50, 5.87, 18],[9.5,9.5,22],[9.5,9.5,25.5]]
        self.margin_error  = 0.8

        self.centroid_list = []
        self.area_list = []
        self.centroids = []  

        self.Kp = [1.95   , 2.49   ,2.5]#previous values - 9.06 , 7.64, 4.8 
        self.Ki = [63*0.0001 , 63*0.0001  , 0.0401]#previous values - 0.0042 ,0.0021, 0.0019
        self.Kd = [69.3   , 72.2 ,168.1 ]#previous values - 423.6 , 165.9 , 102.6 

        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_roll"    ,self.pid_tune_roll_callback,1)
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_pitch"   ,self.pid_tune_pitch_callback,1)
        self.drone_img = node.create_subscription(Image,"/video_frames"   ,self.drone_alien,1)
        
        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)        
        self.integral_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)
        self.rc_command_unfiltered_pub = node.create_publisher(RCMessage, "/luminosity/rc_command_unfiltered",1)

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
        try:
            self.set_points = self.setpoints[self.index]
            print("setpoints ",self.set_points)
            print(self.index+1)
            if float(self.drone_whycon_pose_array.poses[0].position.x)> (self.set_points[0] - self.margin_error) and float(self.drone_whycon_pose_array.poses[0].position.x) < (self.set_points[0] + self.margin_error) :
                if float(self.drone_whycon_pose_array.poses[0].position.y) > (self.set_points[1] - self.margin_error) and float(self.drone_whycon_pose_array.poses[0].position.y)< (self.set_points[1] + self.margin_error) :                
                    if float(self.drone_whycon_pose_array.poses[0].position.z) > (self.set_points[2] - self.margin_error) and float(self.drone_whycon_pose_array.poses[0].position.z) < (self.set_points[2] + self.margin_error) :
                        self.index += 1
                    if self.index >= len(self.setpoints):
                        self.shutdown_hook()
                        self.index = len(self.setpoints) - 1

            self.error[0] = (-(float(self.drone_whycon_pose_array.poses[0].position.x) - self.set_points[0]))
            self.error[1] = (float(self.drone_whycon_pose_array.poses[0].position.y) - self.set_points[1])
            self.error[2] = (float(self.drone_whycon_pose_array.poses[0].position.z) - self.set_points[2])
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
            #1450

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

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()
 
    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    def disarm(self):
        print("disarming")
        throttle = int(self.rc_message.rc_throttle)  
        while throttle >=5 :
            throttle = throttle - 100
            self.publish_data_to_rpi( 1500, 1500, round(throttle) )
            time.sleep(0.8)
            #self.pid()
            print(throttle)
            if throttle <500:
                self.node.get_logger().info("Calling disarm service")
                self.commandbool.value = False
                self.future = self.arming_service_client.call_async(self.commandbool)
    
    def detect_alien(self,img):
        #filtering image
        erosionimg = cv2.erode(img, np.ones((7,7)), iterations=1)
        grayimg = cv2.cvtColor(erosionimg, cv2.COLOR_BGR2GRAY)
        _, thresholdimg = cv2.threshold(grayimg, 180, 255, cv2.THRESH_BINARY)
        dilationimg = cv2.dilate(thresholdimg, np.ones((7,7)), iterations=1)
        #find out counters
        contours = cv2.findContours(dilationimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])
        return contours
    
    def centroid_calculator(self,centroid_list):
        alien =  (len(centroid_list))
        sumofx = sum(xcoor for xcoor , _ in centroid_list)
        sumofy = sum(ycoor for _ , ycoor in centroid_list)
        centroidx = sumofx/alien
        centroidy = sumofy/alien

        if alien == 2 :
            self.centroids.append(['alien_a' , 2 ,centroidx,centroidy])
        elif alien == 3 :
            self.centroids.append(['alien_b' , 3 ,centroidx,centroidy])
        elif alien == 4 :
            self.centroids.append(['alien_c' , 4 ,centroidx,centroidy])
        elif alien == 5 :
            self.centroids.append(['alien_d' , 5 ,centroidx,centroidy])
        elif alien >= 6 :
            centroid_1 = []  # x < 256, y < 256
            centroid_2 = []  # x < 256, y > 256
            centroid_3 = []  # x > 256, y < 256
            centroid_4 = []  # x > 256, y > 256

            for x, y in centroid_list:
                if x < 256 and y < 256:
                    centroid_1.append((x, y))
                elif x < 256 and y > 256:
                    centroid_2.append((x, y))
                elif x > 256 and y < 256:
                    centroid_3.append((x, y))
                else:
                    centroid_4.append((x, y))

            if len(centroid_1) != 0:
                self.centroid_calculator(centroid_1)

            if len(centroid_2) != 0:
                self.centroid_calculator(centroid_2)

            if len(centroid_3) != 0:
                self.centroid_calculator(centroid_3)

            if len(centroid_4) != 0:
                self.centroid_calculator(centroid_4)
        return self.centroids
    
    def write(self,contours,img):
        print(len(contours))
        if len(contours)< 7 and len(contours)> 2:
            for cont in contours:
            # Calculate the area of the contour
                area = cv2.contourArea(cont)
                # Calculate the centroid coordinates
                M = cv2.moments(cont)
                if M['m00'] != 0:
                    cx = float(M['m10'] / M['m00'])
                    cy = float(M['m01'] / M['m00'])
                else:
                    # Handle the case when the area (m00) is zero to avoid division by zero
                    cx, cy = 0, 0

                # Append centroid coordinates and area to the respective lists
                self.area_list.append(area)
                self.centroid_list.append((cx , cy))

            # Draw the bright spot on the image
            cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
            centroid_list = self.centroid_calculator(self.centroid_list)
            print(centroid_list)
            cv2.imshow("output",img)
    def drone_alien(self):
        try:
            drone_ros_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
            print("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////??????")
            contours= self.detect_alien(drone_ros_image)
            self.write(contours,drone_ros_image)
            cv2.imshow("Image window", drone_ros_image)
        except CvBridgeError as e:
          print(e)
    
        
        

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
        # controller.stopFan()
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()