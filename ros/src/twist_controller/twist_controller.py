import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
from std_msgs.msg import Float32

GAS_DENSITY = 2.858
MAX_BRAKE = 400.0


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.heading_angle_sub = rospy.Subscriber('/turning_angle', Float32, self.turning_angle_cb)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # Minimum throttle value
        mx = 0.2  # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.turning_angle = 0

        self.last_time = rospy.get_time()
        self.log_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = MAX_BRAKE  # N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = min(MAX_BRAKE, (abs(decel) * self.vehicle_mass * self.wheel_radius))  # Torque N*m

        if self.turning_angle > 5:
            throttle = min(throttle, 0.01)
            steering = max(-0.3, min(steering, 0.3))
            rospy.logwarn('Turning angle is too big, small throttle')
        return throttle, brake, steering
    
    def turning_angle_cb(self, msg):
        self.turning_angle = msg.data