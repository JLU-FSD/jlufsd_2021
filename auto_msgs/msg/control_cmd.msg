#target throttle in percentage [0, 100]
float64 throttle
#torque for f/r two drivers or single driver
float64 torque_f
float64 torque_r
#torque for four drivers
float64 torque_fl
float64 torque_fr
float64 torque_rl
float64 torque_rr
#target brake in percentage [0, 100]
float64 brake
#brake pressure for four stop
float64 brake_pre_fl
float64 brake_pre_fr
float64 brake_pre_rl
float64 brake_pre_rr
#target steering angle, in percentage of full scale [-100, 100]
float64 steering_target
#lmpc four steering vechicle,target steering angle in dregree
float64 tire_angle_target_f
float64 tire_angle_target_r

float64 tire_angle_target_fl
float64 tire_angle_target_fr
float64 tire_angle_target_rl
float64 tire_angle_target_rr

#control cmd
float32[] tire_angle_cmd
float32[] ad_cmd
float32[] ab_cmd
