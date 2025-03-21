# eyrc_ld_3333

## Description
This is ROS server for controlling the quad drone and perform computer vision tasks while in flight.

## Installation
Instructions on how to install and set up the project.

```bash
# Clone the repository
git clone https://github.com/kedargh/eyrc_ld_3333.git

## Documentation

### Task 5: Waypoint Controller

This script, `LD_3333_waypoint_controller.py`, is a ROS (Robot Operating System) node that controls a drone using PID (Proportional, Integral, Derivative) control to navigate through predefined waypoints.

#### Key Components

- **PID Controller**: The script uses PID control to maintain the drone's position and orientation.
- **WhyCon Pose Subscription**: Subscribes to WhyCon poses to get the drone's current position.
- **RC Commands**: Publishes RC (Remote Control) commands to control the drone's movements.
- **Image Processing**: Processes images to detect and track objects (aliens) using OpenCV.

#### Main Classes and Functions

- **DroneController Class**: Handles the main logic for controlling the drone.
  - `__init__(self, node)`: Initializes the controller.
  - `whycon_poses_callback(self, msg)`: Callback for receiving WhyCon poses.
  - `pid_tune_throttle_callback(self, msg)`, `pid_tune_roll_callback(self, msg)`, `pid_tune_pitch_callback(self, msg)`: Callbacks for tuning PID parameters.
  - `pid(self)`: Executes the PID control loop.
  - `publish_data_to_rpi(self, roll, pitch, throttle)`: Publishes RC commands to the drone.
  - `shutdown_hook(self)`: Handles shutdown procedures.
  - `arm(self)`, `disarm(self)`: Methods to arm and disarm the drone.
  - `detect_alien(self, img)`: Detects objects in images.
  - `centroid_calculator(self, centroid_list)`: Calculates centroids of detected objects.
  - `write(self, contours, img)`: Processes and displays detected objects.
  - `drone_alien(self)`: Main method for processing drone's camera feed and detecting objects.

- **Main Function**: Initializes the ROS node and starts the controller.

```python
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
```

### Task 6: Waypoint Controller

This script, `Task_6.py`, is a ROS (Robot Operating System) node that controls a drone using PID (Proportional, Integral, Derivative) control to navigate through predefined waypoints and detect specific objects (aliens) using image processing techniques.

#### Key Components

- **PID Controller**: The script uses PID control to maintain the drone's position and orientation.
- **WhyCon Pose Subscription**: Subscribes to WhyCon poses to get the drone's current position.
- **RC Commands**: Publishes RC (Remote Control) commands to control the drone's movements.
- **Image Processing**: Processes images to detect and track objects (aliens) using OpenCV.

#### Main Classes and Functions

- **DroneController Class**: Handles the main logic for controlling the drone.
  - `__init__(self, node)`: Initializes the controller.
  - `whycon_poses_callback(self, msg)`: Callback for receiving WhyCon poses.
  - `pid_tune_throttle_callback(self, msg)`, `pid_tune_roll_callback(self, msg)`, `pid_tune_pitch_callback(self, msg)`: Callbacks for tuning PID parameters.
  - `check(self, cal_value, max_value, min_value)`: Checks and limits the calculated values.
  - `saveimg(self, msg)`: Saves ROS image to OpenCV image.
  - `detect_alien(self, img)`: Detects objects in images.
  - `centroid_calculator(self, centroid_list)`: Calculates centroids of detected objects.
  - `write(self, contours, img)`: Processes and displays detected objects.
  - `start_stop_notify(self, delay)`: Publishes RC commands to beep and blink.
  - `beep_alien(self, times)`: Publishes RC commands to beep and blink a specified number of times.
  - `bioloc(self, name)`: Publishes the biolocation of the detected alien.
  - `pid(self)`: Executes the PID control loop.
  - `publish_data_to_rpi(self, roll, pitch, throttle)`: Publishes RC commands to the drone.
  - `shutdown_hook(self)`: Handles shutdown procedures.
  - `arm(self)`, `disarm(self)`: Methods to arm and disarm the drone.

- **Main Function**: Initializes the ROS node and starts the controller.

```python
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
```

## Contributing
Guidelines for contributing to the project.

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes.
4. Commit your changes (`git commit -m 'Add some feature'`).
5. Push to the branch (`git push origin feature-branch`).
6. Open a pull request.

## License
If your project has a license, include the license information here.

## Contact
If you have any questions, feel free to contact me at [kedarkenjalkar@gmail.com].

## Acknowledgments
Contributors other than me - Aditya Khode [https://www.linkedin.com/in/adityakhode10/]
