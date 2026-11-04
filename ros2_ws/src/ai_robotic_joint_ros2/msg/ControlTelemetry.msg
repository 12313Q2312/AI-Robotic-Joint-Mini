# All values are output-side SI units. Time comes from wb_robot_get_time().
float64 sim_time_s
float64 requested_angle_rad
float64 reference_angle_rad
float64 reference_velocity_rad_s
float64 actual_angle_rad
float64 velocity_rad_s
float64 p_torque_nm
float64 i_torque_nm
float64 d_torque_nm
float64 feedforward_torque_nm
float64 commanded_torque_nm
bool saturated
bool anti_windup
uint8 mode
string status
uint64 command_sequence
