import math

class MotionData:
    def __init__(self, pressure_pa, base_pressure, timestep, acc_x, acc_y, acc_z, posi_x, posi_y, posi_z):
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
        self.temp_x, self.temp_y, self.temp_z = 0.0, 0.0, 0.0
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

    def pid_control(self, setpoint, measured, Kp, Ki, Kd, integral, last_error):
        # Calculate error
        error = setpoint - measured
        # Update integral and derivative terms
        integral += error * self.dt
        derivative = (error - last_error) / self.dt
        # PID output
        output = Kp * error + Ki * integral + Kd * derivative
        return output, error, integral

    def servo_stabilise_Y(self, pos_y):
        """
        Stabilize rotation along Y-axis using PID.
        """
        setpoint = 0  # Target position for level orientation
        pid_output, self.error_y, self.integral_y = self.pid_control(
            setpoint, pos_y, self.Kp_y, self.Ki_y, self.Kd_y, self.integral_y, self.error_y)
        
        # Adjust servo duty based on PID output
        duty = self.servo_duty_base + pid_output
        return [duty, duty]  # Left and right servo duty for Y-axis

    def servo_stabilise_Z(self, pos_z):
        """
        Stabilize rotation along Z-axis using PID.
        """
        setpoint = 0  # Target position for level orientation
        pid_output, self.error_z, self.integral_z = self.pid_control(
            setpoint, pos_z, self.Kp_z, self.Ki_z, self.Kd_z, self.integral_z, self.error_z)
        
        # Adjust servo duty based on PID output
        duty_left = self.servo_duty_base + pid_output
        duty_right = self.servo_duty_base - pid_output
        return [duty_left, duty_right]  # Left and right servo duty for Z-axis

    def motor_stabilise_X(self, pos_x):
        """
        Stabilize the motor along X-axis using PID.
        """
        setpoint = 0  # Target position for level orientation
        pid_output, self.error_x, self.integral_x = self.pid_control(
            setpoint, pos_x, self.Kp_x, self.Ki_x, self.Kd_x, self.integral_x, self.error_x)
        
        # Adjust motor duty based on PID output
        duty_base = 100  # Base motor duty
        duty_left = duty_base + pid_output
        duty_right = duty_base - pid_output
        return [duty_left, duty_right]  # Left and right motor duty for X-axis
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
    
    def decode_pos(self,pos_x,pos_y,pos_z): # for obtaining actual position! main code needs treshold, as gyro is floating plus-minus 2% of 360 [deg/s]
        # yeah, gyro gives data in [deg/s], so it detects change only. hence, to obtain actual position - just a simple numerical integration of data
        self.prev_x=self.temp_x
        self.prev_y=self.temp_y
        self.prev_z=self.temp_z
        self.temp_x+=pos_x*self.dt
        self.temp_y+=pos_y*self.dt
        self.temp_z+=pos_z*self.dt
        
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
