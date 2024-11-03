import math

import math
import time

class MadgwickAHRS:
    def __init__(self, beta=0.1, sample_freq=50.0):
        self.beta = beta
        self.sample_freq = sample_freq
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # Convert gyroscope degrees/sec to radians/sec
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return  # avoid division by zero
        ax, ay, az = ax / norm, ay / norm, az / norm

        # Auxiliary variables
        _2q0mx, _2q1mx, _2q2mx, _2q3mx = 2 * q0, 2 * q1, 2 * q2, 2 * q3
        _4q0, _4q1, _4q2 = 4 * q0, 4 * q1, 4 * q2
        _8q1, _8q2 = 8 * q1, 8 * q2

        # Gradient descent algorithm corrective step
        s0 = _4q0 * q0 * q0 - _2q1mx + _4q0 * q1 * q1 - ax
        s1 = _4q1 * q0 * q0 + _4q1 * q1 * q1 - ay
        s2 = _4q2 * q0 * q0 + _4q2 * q1 * q1 - az
        s3 = _8q1 * q1 * q3 + _8q2 * q2 * q3

        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  # Normalize step magnitude
        s0, s1, s2, s3 = s0 / norm, s1 / norm, s2 / norm, s3 / norm

        # Apply feedback step
        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3

        # Integrate to yield quaternion
        q0 += qDot0 / self.sample_freq
        q1 += qDot1 / self.sample_freq
        q2 += qDot2 / self.sample_freq
        q3 += qDot3 / self.sample_freq

        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q0, self.q1, self.q2, self.q3 = q0 / norm, q1 / norm, q2 / norm, q3 / norm

    def get_euler_angles(self):
        # Calculate pitch, roll, yaw from quaternion
        pitch = math.degrees(math.asin(2 * (self.q0 * self.q2 - self.q3 * self.q1)))
        roll = math.degrees(math.atan2(2 * (self.q0 * self.q1 + self.q2 * self.q3), 1 - 2 * (self.q1 * self.q1 + self.q2 * self.q2)))
        yaw = math.degrees(math.atan2(2 * (self.q1 * self.q2 + self.q0 * self.q3), 1 - 2 * (self.q2 * self.q2 + self.q3 * self.q3)))
        return pitch, roll, yaw

class MotionData:
    def __init__(self, pressure_pa, base_pressure, timestep, acc_x, acc_y, acc_z, posi_x, posi_y, posi_z):
        madgwick = MadgwickAHRS()
        # Initialize servo and motor properties
        self.servo_freq = 50
        self.servo_duty_base = 1.5

        # Initialize PID parameters for each axis
        self.Kp_x, self.Ki_x, self.Kd_x = 1.0, 0.0, 0.1
        self.Kp_y, self.Ki_y, self.Kd_y = 1.0, 0.0, 0.1
        self.Kp_z, self.Ki_z, self.Kd_z = 1.0, 0.0, 0.1
        
        # Initialize error values and integral terms for PID
        self.error_x, self.error_y, self.error_z = 0, 0, 0
        self.integral_x, self.integral_y, self.integral_z = 0, 0, 0
        self.last_error_x, self.last_error_y, self.last_error_z = 0, 0, 0

        # Other properties and accelerations
      
        self.press = pressure_pa
        self.base_p = base_pressure * 100
        self.dt = timestep

        # Correct accelerations to make unit vector approximately 1.0
        self.accel_corr_factor = abs(1 - math.sqrt(acc_x ** 2 + acc_y ** 2 + acc_z ** 2))
        self.ax = acc_x * 9.8105 / self.accel_corr_factor
        self.ay = acc_y * 9.8105 / self.accel_corr_factor
        self.az = acc_z * 9.8105 / self.accel_corr_factor

        # Convert positions from radians to degrees
        self.pos_x = posi_x / (2 * math.pi) * 360 if posi_x > 5/(2 * math.pi) * 360.0 else posi_x
        self.pos_y = posi_y / (2 * math.pi) * 360 if posi_y > 5/(2 * math.pi) * 360.0 else posi_y
        self.pos_z = posi_z / (2 * math.pi) * 360 if posi_z > 5/(2 * math.pi) * 360.0 else posi_z
        self.corr_pos = math.sqrt(posi_x ** 2 + posi_y ** 2 + posi_z ** 2)
        #madgwick init
		madgwick.update(self.pos_x, self.pos_y, self.pos_z, self.ax, self.ay, self.az)
    	pitch, roll, yaw = madgwick.get_euler_angles()
    	self.temp_x, self.temp_y, self.temp_z = pitch,roll,yaw
    	'''
    def pid_control(self, setpoint, measured, Kp, Ki, Kd, integral, last_error):
        # Calculate error
        error = setpoint - measured
        # Update integral and derivative terms
        integral += error * self.dt
        derivative = (error - last_error) / self.dt
        # PID output
        output = Kp * error + Ki * integral + Kd * derivative
        return output, error, integral
'''
    def servo_stabilise_Y(self, pos_y):
        """
        Stabilize rotation along Y-axis.
        """
        if self.prev_y - pos_y < 0:
            return [(1.5 + 0.5 * (1 - math.exp(-(self.prev_y - pos_y)))),
                    (1.5 +0.5 * (1 - math.exp(-(self.prev_y - pos_y))))]
        else:
            return [(1.5 - 0.5 * (1 - math.exp(-(self.prev_y - pos_y)))),
                    (1.5 - 0.5 * (1 - math.exp(-(self.prev_y - pos_y))))]

    def servo_stabilise_Z(self, pos_z):
        """
        Stabilize rotation along Z-axis.
        """
        if self.prev_z - pos_z < 0:
            return [(1.5 - 0.5 * (1 - math.exp(-(self.prev_z - pos_z)))),
                    (1.5 + 0.5 * (1 - math.exp(-(self.prev_z - pos_z))))]
        else:
            return [(1.5 + 0.5 * (1 - math.exp(-(self.prev_z - pos_z)))),
                    (1.5 - 0.5 * (1 - math.exp(-(self.prev_z - pos_z))))]

    def motor_stabilise_X(self, pos_x):
        """
        Stabilize the motor along X-axis.
        """
        duty = 100
        if self.prev_x - pos_x < 0:
            return [duty * (1 + math.exp(-(self.prev_x - pos_x))),
                    duty * (1 - math.exp(-(self.prev_x - pos_x)))]
        else:
            return [duty * (1 - math.exp(-(self.prev_x - pos_x))),
                    duty * (1 + math.exp(-(self.prev_x - pos_x)))]
    def decode_acc(self,pos_x,pos_y,pos_z,accx,accy,accz):#those are actual orientations of gyro, not base, as well as accelerations
        g=9.8105
        # rotation vec now... rotvec=[sin(pos_z)*cos(pos_y),sin(pos_x)*cos(pos_z),sin(pos_y)*cos(pos_x)]
        x=math.sin(pos_z)*math.cos(pos_y) # nice symmetry. obtained by writing all 3 equations for all 3 possible rotations - then multiplying each one
        y=math.sin(pos_x)*math.cos(pos_z)
        z=math.sin(pos_y)*math.cos(pos_x)

        gx=g*x # maybe wrong, i'll test. 03.06.24: it is fine
        gy=g*y
        gz=g*z
        return (accx-gx,accy-gy,accz-gz) # and we have drone accels!
    
    def decode_pos(self,pos_x,pos_y,pos_z,ax,ay,az): # for obtaining actual position
        self.prev_x=self.temp_x
        self.prev_y=self.temp_y
        self.prev_z=self.temp_z
        madgwick.update(pos_x, pos_y, pos_z, ax, ay, az)
    	pitch, roll, yaw = madgwick.get_euler_angles()
        self.temp_x=pitch
        self.temp_y=roll
        self.temp_z=yaw
        
        return [self.temp_x,self.temp_y,self.temp_z]
    @property
    def upwards(self,signal_up):
    	duty=100
    	return duty*signal_up
    @property
	def downwards(self,signal_down):
		duty=100
		return duty/signal_down
	@property
	def straight(self,signal_straight):
		duty=100
		servo=2
		return [duty*signal_straight,servo]
	@property
	def backwards(self,signal_back):
		duty=100
		servo=1
		return [duty/signal_back,servo]
	@property
	def rotate_l(self, signal_l):
		duty=100
		servo1=1.25
		servo2=1.75
		return [duty*signal_l,servo1,servo2]
	@property
	def rotate_r(self,signal_r):
		duty=100
		servo1=1.75
		servo2=1.25
		return [duty*signal_l,servo1,servo2]
