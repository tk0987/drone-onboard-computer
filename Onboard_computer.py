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
