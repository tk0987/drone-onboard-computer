import math

class MotionData:
    def __init__(self, pressure_pa, base_pressure, timestep, acc_x, acc_y, acc_z, posi_x, posi_y, posi_z):
        self.servo_freq = 50
        self.servo_duty_base = 1.5
        

        self.temp_x = 0.0
        self.temp_y = 0.0
        self.temp_z = 0.0
        self.press = pressure_pa
        self.base_p = base_pressure * 100
        self.dt = timestep

        # Correct accelerations to make unit vector approximately 1.0
        self.accel_corr_factor = (1 - math.sqrt(acc_x ** 2 + acc_y ** 2 + acc_z ** 2))
        self.ax = acc_x * 9.8105 / self.accel_corr_factor
        self.ay = acc_y * 9.8105 / self.accel_corr_factor
        self.az = acc_z * 9.8105 / self.accel_corr_factor

        # Convert positions from radians to degrees
        if posi_x>5:
        	self.pos_x = posi_x / (2 * math.pi) * 360.0
		if posi_y>5:
        	self.pos_y = posi_y / (2 * math.pi) * 360.0
        if posi_z>5:
        	self.pos_z = posi_z / (2 * math.pi) * 360.0
        self.corr_pos = math.sqrt(posi_x ** 2 + posi_y ** 2 + posi_z ** 2)

    @property
    def altitude(self):
        """
        Calculate altitude based on actual pressure, ground-level pressure, and temperature.
        """
        g = 9.8105
        R = 8.314
        u = 0.0289644
        temp = self.temp_c + 273.15  # Example, assuming self.temp_c exists
        press = self.press * 100  # Convert from hPa to Pa
        press_0 = self.base_p  # Ground-level pressure in Pa
        return math.log(press_0 / press) * R * temp / (u * g)

    def velocity(self, accx, accy, accz):
        return (accx * self.dt, accy * self.dt, accz * self.dt)

    def distance(self, accx, accy, accz):
        return (accx * self.dt ** 2, accy * self.dt ** 2, accz * self.dt ** 2)

    def decode_acc(self, pos_x, pos_y, pos_z, accx, accy, accz):
        """
        Decode the accelerations based on actual orientations of gyro.
        """
        g = 9.8105
        x = math.sin(pos_z) * math.cos(pos_y)
        y = math.sin(pos_x) * math.cos(pos_z)
        z = math.sin(pos_y) * math.cos(pos_x)

        gx = g * x
        gy = g * y
        gz = g * z
        return (accx - gx, accy - gy, accz - gz)

    def decode_pos(self, pos_x, pos_y, pos_z):
        """
        Integrate gyro data to obtain actual position.
        """
        self.prev_x = self.temp_x
        self.prev_y = self.temp_y
        self.prev_z = self.temp_z
        self.temp_x += pos_x * self.dt
        self.temp_y += pos_y * self.dt
        self.temp_z += pos_z * self.dt

        return [self.temp_x, self.temp_y, self.temp_z]

    def servo_stabilise_Y(self, pos_y):
        """
        Stabilize rotation along Y-axis.
        """
        if self.prev_y - pos_y < 0:
            return [(1.5 + 0.5 * (1 - math.exp(-(self.prev_y - pos_y)))),
                    (1.5 - 0.5 * (1 - math.exp(-(self.prev_y - pos_y))))]
        else:
            return [(1.5 - 0.5 * (1 - math.exp(-(self.prev_y - pos_y)))),
                    (1.5 + 0.5 * (1 - math.exp(-(self.prev_y - pos_y))))]

    def servo_stabilise_x(self, pos_z):
        """
        Stabilize rotation along X-axis.
        """
        if self.prev_z - pos_z < 0:
            return [(1.5 - 0.5 * (1 - math.exp(-(self.prev_z - pos_z)))),
                    (1.5 - 0.5 * (1 - math.exp(-(self.prev_z - pos_z))))]
        else:
            return [(1.5 + 0.5 * (1 - math.exp(-(self.prev_z - pos_z)))),
                    (1.5 + 0.5 * (1 - math.exp(-(self.prev_z - pos_z))))]

    def motor_stabilise_X(self, pos_x):
        """
        Stabilize the motor along X-axis.
        """
        duty = 100
        if self.prev_x - pos_x < 0:
            return [duty * (1 + math.exp(-(self.prev_x - pos_x))),
                    duty * (1 + math.exp(-(self.prev_x - pos_x)))]
        else:
            return [duty * (1 - math.exp(-(self.prev_x - pos_x))),
                    duty * (1 - math.exp(-(self.prev_x - pos_x)))]

    @property
    def upwards(self):
        duty = 100
        return duty * self.signal_up

    @property
    def downwards(self):
        duty = 100
        return duty / self.signal_down

    @property
    def straight(self):
        duty = 100
        servo = 2
        return [duty * self.signal_straight, servo]

    @property
    def backwards(self):
        duty = 100
        servo = 1
        return [duty / self.signal_back, servo]

    @property
    def rotate_l(self):
        duty = 100
        servo1 = 1.25
        servo2 = 1.75
        return [duty * self.signal_l, servo1, servo2]

    @property
    def rotate_r(self):
        duty = 100
        servo1 = 1.75
        servo2 = 1.25
        return [duty * self.signal_r, servo1, servo2]
