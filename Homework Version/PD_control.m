function u = PD_control(q_des, q_robot, PERIOD)
persistent yaw_des_prev yaw_robot_prev firstRun

if isempty(firstRun)
    yaw_des_prev   = 0;
    yaw_robot_prev = 0;
    
    firstRun = 1;
end

yaw_des =
yaw_robot =

yaw_des_dot =
yaw_robot_dot =

e_phi =
e_phi_dot =
e_phi_int =

yaw_des_prev = 
yaw_robot_prev = 

u = feedforward + e_phi*Kp + e_phi_dot*Kd + e_phi_int*Ki;