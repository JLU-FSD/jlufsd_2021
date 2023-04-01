float64 speed_mps
uint8 driving_mode
uint8 gear_location
#lmpc four steering vechicle
float64 steering_angle_f
float64 steering_angle_r

float64 steering_angle_fl
float64 steering_angle_fr
float64 steering_angle_rl
float64 steering_angle_rr

float64 mc_pressure

#brake pressure for four stop
float64 brake_pre_fl
float64 brake_pre_fr
float64 brake_pre_rl
float64 brake_pre_rr

#torque for f/r two drivers or single driver
float64 torque_f
float64 torque_r

#torque for four drivers
float64 torque_fl
float64 torque_fr
float64 torque_rl
float64 torque_rr

#rote_speed
float64 rote_speed_fl
float64 rote_speed_fr
float64 rote_speed_rl
float64 rote_speed_rr

# driving acc
float32[] ad_fb
# braking acc
float32[] ab_fb
# steering_angle
float32[] steering_angle_fb
