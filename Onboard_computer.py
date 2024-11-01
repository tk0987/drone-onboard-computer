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
