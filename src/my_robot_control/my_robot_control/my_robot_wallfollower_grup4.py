#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Paràmetres
        self.declare_parameter('distance_limit', 0.30)   # distància desitjada a la paret dreta
        self.declare_parameter('forward_speed', 0.40)    # velocitat lineal base
        self.declare_parameter('lateral_gain', 0.6)      # guany per corregir lateral (vy = -gain * error)
        self.declare_parameter('time_to_stop', 30.0)     # aturada automàtica
        self.declare_parameter('tolerance', 0.04)        # banda de tolerància al voltant de distance_limit

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.k_lat = float(self.get_parameter('lateral_gain').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Últim Twist ordenat (es publicarà periòdicament)
        self.cmd = Twist()

        # Entitats ROS 2
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)  # 10 Hz

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("WallFollower holonòmic: control lateral i diagonals (sense gir).")

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Sectors (radiants convertits a graus). Ajusta si cal.
        FRONT, FR_RIGHT, RIGHT, BACK_RIGHT, BACK = [], [], [], [], []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -10 <= ang <= 10:
                FRONT.append(d)
            elif -70 <= ang < -10:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)

        # Distàncies mínimes
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')

        twist = Twist()
        action = ""

        # Regla de prioritat: tria el sector amb obstacle més proper
        sectors = {
            "FRONT": min_front,
            "FR_RIGHT": min_fr_right,
            "RIGHT": min_right,
            "BACK_RIGHT": min_back_right,
            "BACK": min_back,
        }
        closest_region = min(sectors, key=sectors.get)
        closest_dist = sectors[closest_region]

        # Moviment holonòmic pur (sense gir): linear.x endavant/enrere, linear.y esquerra/dreta
        # 1) FRONT → moure cap a l’esquerra (evitar xocar)
        if closest_region == "FRONT" and closest_dist < self.base_distance:
            twist.linear.x = 0.0
            twist.linear.y = +self.v_lin
            action = f"FRONT {min_front:.2f} m → LEFT"

        # 2) FRONT-RIGHT → diagonals cap endavant-esquerra
        elif closest_region == "FR_RIGHT" and closest_dist < self.base_distance:
            twist.linear.x = +self.v_lin * 0.6
            twist.linear.y = +self.v_lin * 0.6
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → FRONT-LEFT"

        # 3) RIGHT visible → mantenir orientació paral·lela i distància amb control lateral
        elif math.isfinite(min_right):
            error = min_right - self.base_distance  # >0 massa lluny; <0 massa a prop
            # Banda morta per evitar oscil·lacions
            if abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m ≈ {self.base_distance:.2f}±{self.tol:.2f}) → STRAIGHT"
                )
            else:
                # Correcció lateral proporcional (cap a la paret si massa lluny, allunya si massa a prop)
                twist.linear.x = self.v_lin
                twist.linear.y = -self.k_lat * error
                # Saturació suau de vy
                max_lat = self.v_lin
                if twist.linear.y >  max_lat: twist.linear.y =  max_lat
                if twist.linear.y < -max_lat: twist.linear.y = -max_lat
                dir_txt = "LEFT" if twist.linear.y > 0 else "RIGHT"
                action = (
                    f"RIGHT error {error:+.2f} m → forward + lateral {dir_txt} (vy={twist.linear.y:.2f})"
                )

        # 4) BACK-RIGHT → diagonals cap endavant-dreta (reconnectar amb la paret dreta)
        elif math.isfinite(min_back_right):
            twist.linear.x = +self.v_lin * 0.6
            twist.linear.y = -self.v_lin * 0.6
            action = f"BACK-RIGHT {min_back_right:.2f} m → FRONT-RIGHT"

        # 5) BACK → moure cap a la dreta (allunyar obstacle del darrere)
        elif math.isfinite(min_back):
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin
            action = f"BACK {min_back:.2f} m → RIGHT"

        # 6) Sense deteccions fiables → avanç moderat
        else:
            twist.linear.x = self.v_lin * 0.6
            twist.linear.y = 0.0
            action = "No wall reliably detected → slow forward"

        # Cap gir en cap cas
        twist.angular.z = 0.0

        # Actualitza l’últim comandament
        self.cmd = twist

        # Log (només si canvia l’acció)
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action
        self._state_action = action

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
