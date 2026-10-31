#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from collections import deque
from statistics import mean


DATARATE = 50


class PID:
    def __init__(self, time, logger):
        self.kp = 0.98
        self.ki = 0.01
        self.kd = 0.2

        self.prev_error = 0.0
        self.integral = 0.0

        self.max_omega = 2.5
        self.base_speed = 0.18

        self.last_known_direction = 0  # +1 for left, -1 for right
        self.last_time = time
        self.dt = 0

        self.logger = logger

        self.reset_wiggle()
    
    def controller(self, L, C, R, time):
        """returns (linear, angular)"""
        dt = (time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.dt = dt
        self.last_time = time

        total_detected = L + C + R

        if total_detected > 0: # atleast one sensor on line
            self.reset_wiggle()
            
            error = ((1 * L) + (0 * C) + (-1 * R)) / total_detected # normalized

            if error > 0:
                self.last_known_direction = 1  # left
            elif error < -0:
                self.last_known_direction = -1 # right
            
            self.integral += error * dt
            self.integral = max(min(self.integral, 1), -1)

            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            # u(t) = Kp*e + Ki*integral + Kd*derivative
            angular_z = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            angular_z = max(min(angular_z, self.max_omega), -self.max_omega)

            linear = self.base_speed
        else:
            linear, angular_z = self.do_wiggle()

        return linear, angular_z

    def reset_wiggle(self):
        self.wiggle_fwd_dist = 0.030
        self.wiggle_max_angle = 0.785398163 # ~~30 deg, (min 25.81358 deg)~~ 45deg

        self.wiggle_linvel = 0.03
        self.wiggle_angvel = self.max_omega * 0.54

        self.wiggle_time = 0.0
        self.wiggle_time_runfor = 0.0
        self.wiggle_direction = 1 # 1 left, -1 right
        self.wiggle_stage = 0 # 0: reattempt dirn, 1: linear fwd, 2: angle dir1, 3: angle dir2

    def do_wiggle(self):
        self.logger.info(f"Wiggle time: {self.wiggle_time}")
        if self.wiggle_stage == 0:
            if self.wiggle_time_runfor == 0:
                self.logger.info("!!!!!!!!! WIGGLE START !!!!!!!!")
                self.wiggle_time_runfor = 1.0

            if self.wiggle_time < self.wiggle_time_runfor:
                self.wiggle_time += self.dt
                return 0.03, self.last_known_direction * (self.max_omega * 0.65)
            else:
                self.wiggle_stage = 1
                self.logger.info("!!!!!!!!! SET WIGGLE STAGE 1 !!!!!!!!")
                self.wiggle_time = 0
                self.wiggle_time_runfor = 0

                return 0.0, 0.0
            

        if self.wiggle_stage == 1:
            if self.wiggle_time_runfor == 0:
                self.wiggle_time_runfor = self.wiggle_fwd_dist / self.wiggle_linvel
                self.logger.info(f"=== Expected {self.wiggle_fwd_dist} in {self.wiggle_time_runfor}")

            if self.wiggle_time < self.wiggle_time_runfor:
                self.wiggle_time += self.dt
                return self.wiggle_linvel, 0.0
            else:
                self.logger.info("!!!!!!!!! SET WIGGLE STAGE 2 !!!!!!!!")
                self.wiggle_direction = self.last_known_direction
                self.wiggle_stage = 2
                self.wiggle_time = 0
                self.wiggle_time_runfor = 0

                return 0.0, 0.0
            
        elif self.wiggle_stage == 2:
            if self.wiggle_time_runfor == 0:
                self.wiggle_time_runfor = self.wiggle_max_angle / self.wiggle_angvel
                self.logger.info(f"=== Expected {self.wiggle_max_angle} in {self.wiggle_time_runfor}")

            if self.wiggle_time < self.wiggle_time_runfor:
                self.wiggle_time += self.dt
                return 0.0, self.wiggle_direction * self.wiggle_angvel
            else:
                self.logger.info("!!!!!!!!! SET WIGGLE STAGE 3 !!!!!!!!")
                self.wiggle_stage = 3
                self.wiggle_time = 0
                self.wiggle_time_runfor = 0
                self.wiggle_direction *= -1

                return 0.0, 0.0

        elif self.wiggle_stage == 3:
            if self.wiggle_time_runfor == 0:
                self.wiggle_time_runfor = (self.wiggle_max_angle / self.wiggle_angvel) * 2.5
                self.logger.info(f"=== Expected {self.wiggle_max_angle * 2} in {self.wiggle_time_runfor}")


            if self.wiggle_time < self.wiggle_time_runfor:
                self.wiggle_time += self.dt
                return 0.0, self.wiggle_direction * self.wiggle_angvel
            else:
                self.logger.info("&&&&&&&&&&&&& wiggle is complete what do i do idk")
                self.reset_wiggle()
                return 0.0, 0.0
        return 0.0, 0.0


class IR:
    def __init__(self, ir_name: str):
        self.online = 0
        self.avg_q = deque([0.014], maxlen=6)
        self.frame_id = "ir_line_follower/base_footprint/" + ir_name


class LineFollower(Node):
    def __init__(self, node_name="pything", *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)

        self.ir_range_threshold = 0.0145 # 0.014 on line, 0.015 off line

        self.prev_error = 0.0
        self.integral = 0.0

        self.sub_l = self.create_subscription(LaserScan, '/ir_left', self.ir_callback, 10)
        self.sub_c = self.create_subscription(LaserScan, '/ir_center', self.ir_callback, 10)
        self.sub_r = self.create_subscription(LaserScan, '/ir_right', self.ir_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ir_l = IR("ir_left")
        self.ir_c = IR("ir_center")
        self.ir_r = IR("ir_right")

        self.last_known_direction = 0.0  # +1.0 for left, -1.0 for right
        self.last_time = self.get_clock().now()

        self.pid = PID(self.get_clock().now(), self.get_logger())

        self.timer = self.create_timer(1/DATARATE, self.control_loop)
        self.get_logger().info("control_loop isready " + str(self.timer.is_ready()))
        self.get_logger().info("!!!!!!!!!!!! LineFollower node started !!!!!!!!!!!!")

    
    def ir_callback(self, msg: LaserScan):
        # if (msg.header.frame_id == "ir_line_follower/base_footprint/ir_center"):
        #     pass
            # self.get_logger().info("*"*20 + msg.__repr__())
            # self.get_logger().info("aaaaaaaaaaaaaaaaaaa"*32 + "\t\t" + str(round(msg.ranges[0], 5)))
            # self.avg_q.append(msg.ranges[0])
            # self.get_logger().info("aaaaaaaaaaaaaaaaaaa"*32 + "\t\t" + str(round(mean(self.avg_qc), 6)))

        thresh = self.ir_range_threshold

        if msg.header.frame_id == self.ir_l.frame_id:
            self.ir_l.avg_q.append(msg.ranges[0])
            dist = mean(self.ir_l.avg_q)
            self.ir_l.online = int(dist < thresh)

        elif msg.header.frame_id == self.ir_c.frame_id:
            self.ir_c.avg_q.append(msg.ranges[0])
            dist = mean(self.ir_c.avg_q)
            self.ir_c.online = int(dist < thresh)

        elif msg.header.frame_id == self.ir_r.frame_id:
            self.ir_r.avg_q.append(msg.ranges[0])
            dist = mean(self.ir_r.avg_q)
            self.ir_r.online = int(dist < thresh)

    def control_loop(self):
        cmd = Twist()
        linear, angular = self.pid.controller(
            self.ir_l.online, self.ir_c.online, self.ir_r.online, self.get_clock().now()
        )

        self.get_logger().info(f"%%%\t{self.ir_l.online} {self.ir_c.online} {self.ir_r.online}\t{round(linear, 3)} : {round(angular, 3)}\tlastdir: {self.pid.last_known_direction}, dt:{round(self.pid.dt, 4)}")

        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)
        


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower("pything")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Twist()
        node.cmd_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
