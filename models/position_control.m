function [phi_d,theta_d,psi_d] = position_control(state,model_n_control_param)
m = model_n_control_param(1)
g = model_n_control_param(2)
phi = state(7)
theta = state(8)
psi = state(9)
z

