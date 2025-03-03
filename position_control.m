function [phi_d,theta_d,psi_d] = position_control(x_d,y_d,z_d,state)
run('model_n_control_param.m');
run('traj_inizialization.m'); % Return pos,vel,acc

phi = state(7);
theta = state(8);
psi = state(9);
x = state(1);
y = state(2);
z = state(3);




