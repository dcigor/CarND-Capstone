import rospy

from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

#Long PID Parameters
LON_KP = 2.0
LON_KI = 0.01
LON_KD = 0.0
LON_MIN = -10.	#max deceleration
LON_MAX = 100.	#max acceleration (max throttle position [%])


class Controller(object):
    def __init__(self, m, fc, wb, wr, sr, min_speed, max_lat_accel, max_steer_angle, brake_deadband):
        # TODO: Implement
	self.brakedeadband = brake_deadband
	self.m = m
	self.fc = fc
	self.wr = wr

	self.longPID = PID(LON_KP, LON_KI, LON_KD, LON_MIN, LON_MAX)

	self.SteeringControl = YawController(wb, sr, min_speed, max_lat_accel, max_steer_angle)

	self.act_brake = 0.
	self.act_throttle = 0.
	self.act_steering = 0.

        pass

    def control(self, targetLongSpeed, targetYawrate, current_speed, dbw_enabled, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer


	#If DBW is not active, actuation request shall be inhibited	
	if dbw_enabled == False:	
		self.longPID.reset()
		return 0.,0.,0.

	### LONGITUDINAL

	#Calculate the error speed between the ego vehicle speed and the set speed
	error_speed = targetLongSpeed - current_speed	

	#Calculate the actuation due to the speed error
	acc_cmd = self.longPID.step(error_speed, dt)

	#If the command is positive, the car shall apply throttle
	if(acc_cmd > 0.0):
		self.act_throttle = acc_cmd
		self.act_brake = 0.0

	else:
		#If the command is negative, not throttle is required
		self.act_throttle = 0.0
		
		#IF the command is higher than the breakdeadband, brakes shall be applied.
		if(abs(acc_cmd)> self.brakedeadband):
			self.act_brake = abs(acc_cmd)* (self.m + self.fc*GAS_DENSITY) * self.wr
		else:
			self.act_brake = 0.0

	if((targetLongSpeed == 0.0)&(current_speed < 1.0)): #If the vehicle is stationary and the target speed is 0, the vehicle shall apply brake actuation.
		self.act_brake = 5.0* (self.m + self.fc*GAS_DENSITY) * self.wr
		self.act_throttle = 0.0
		self.longPID.reset()

	### LATERAL

	#Calculate the required steering angle
	self.act_steering = self.SteeringControl.get_steering(targetLongSpeed, targetYawrate, current_speed)	


	#rospy.logerr("TS: %.1f VS: %.1f ||||| ACC: %.1f ||||| T %.1f | B %.1f | St %.1f", targetLongSpeed/ONE_MPH, current_speed/ONE_MPH, acc_cmd, self.act_throttle, self.act_brake, self.act_steering)

	return self.act_throttle, self.act_brake, self.act_steering

