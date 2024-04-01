"""basic motion data processing, from gyro/accel. 
I use mpu6050 with bmp180 module. 
This class is quite general, so it does not matter"""
# import utime
import math
class motion_data:
    def __init__(self,pressure_pa,base_pressure,timestep,acc_x,acc_y,acc_z,posi_x,posi_y,posi_z):
        self.temp_x=0.0
        self.temp_y=0.0
        self.temp_z=0.0
        self.press=pressure_pa
        self.base_p=base_pressure*100
        # self.temp=temp_c
        self.dt=timestep
        # note: accelerations in g units does not create unit vector of length 1.0. typically it is like 0.94. so - corrections. I mean - acc_i are 'base' accelerations, at start:
        self.accel_corr_factor=(1-math.sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z))
        self.ax=acc_x*9.8105/self.accel_corr_factor
        self.ay=acc_y*9.8105/self.accel_corr_factor
        self.az=acc_z*9.8105/self.accel_corr_factor
        # note: positions are in [rad/s], but from -2PI to +2PI... all three axes
        # again: making unity (gyro/accel sucks)
        # new one thing: 
        self.pos_x=posi_x/2/math.pi*360.0
        self.pos_y=posi_y/2/math.pi*360.0
        self.pos_z=posi_z/2/math.pi*360.0
        self.corr_pos=math.sqrt(posi_x*posi_x+posi_y*posi_y+posi_z*posi_z) # for unity making
        # self.ox=pos_x/corr_pos # all in [rad]!!!
        # self.oy=pos_y/corr_pos
        # self.oz=pos_z/corr_pos
    @property
    def altitude(self,press,press_0,temp):
        """
        we need: actual pressure, ground level pressure, temperature also.
        i started with wiki equation: press=press_0*exp(-u*g*h/(R*T)),
        where press = actual pressure, press_0 - pressure at ground level, u - molar mass of air: 0.0289644 kg/mol, g - gravit. accel., R = 8.314 J/(kg*K), T= temp. in [K]
        resulting in equation:
        h=ln(press_0-press)*R*temp/(u*g)
        """
        g=9.8105 # m/s^2; value for cracov... not important, circa 9.8
        R=8.314
        u=0.0289644
        temp=temp+273.15 # kelvins from *C
        press=press*100 # from hPa into Pa
        press_0=self.base_p # in Pa
        return math.log(press_0-press)*R*temp/(u*g)
    def velocity(self,accx,accy,accz):
        return (accx*self.dt,accy*self.dt,accz*self.dt)
    def distance(self,accx,accy,accz):
        return (accx*self.dt*self.dt,accy*self.dt*self.dt,accz*self.dt*self.dt)
    @property
    def decode_acc(self,pos_x,pos_y,pos_z,accx,accy,accz):#those are actual orientations of gyro, not base, as well as accelerations
        g=9.8105
        # rotation vec now... rotvec=[sin(pos_z)*cos(pos_y),sin(pos_x)*cos(pos_z),sin(pos_y)*cos(pos_x)]
        x=math.sin(pos_z)*math.cos(pos_y) # nice symmetry. obtained by writing all 3 equations for all 3 possible rotations - then multiplying each one
        y=math.sin(pos_x)*math.cos(pos_z)
        z=math.sin(pos_y)*math.cos(pos_x)

        gx=g*x # maybe wrong, i'll test
        gy=g*y
        gz=g*z
        return (accx-gx,accy-gy,accz-gz) # and we have drone accels!
    
    def decode_pos(self,pos_x,pos_y,pos_z): # for obtaining actual position! main code needs treshold, as gyro is floating plus-minus 2% of 360 [deg/s]
        # yeah, gyro gives data in [deg/s], so it detects change only. hence, to obtain actual position - just a simple numerical integration of data
        self.temp_x+=pos_x*self.dt
        self.temp_y+=pos_y*self.dt
        self.temp_z+=pos_z*self.dt
        return [self.temp_x,self.temp_y,self.temp_z]
