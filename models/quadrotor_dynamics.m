function dpdt = quadrotor_dynamics(t, p, u)
    % p: stato del drone [x; y; z; phi; theta; psi; dx; dy; dz; dphi; dtheta; dpsi]
    % u: input di controllo [T; tau_phi; tau_theta; tau_psi]
    
    % Parametri del drone
    m = 1.0;  % massa
    g = 9.81; % accelerazione gravitazionale
    
    % Estrai gli stati
    x = p(1); y = p(2); z = p(3);
    phi = p(4); theta = p(5); psi = p(6);
    dx = p(7); dy = p(8); dz = p(9);
    dphi = p(10); dtheta = p(11); dpsi = p(12);
    
    % Estrai gli input
    T = u(1); tau_phi = u(2); tau_theta = u(3); tau_psi = u(4);
    
    % Equazioni dinamiche
    ddx = (T/m) * (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    ddy = (T/m) * (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
    ddz = (T/m) * cos(theta)*cos(phi) - g;
    
    % Derivative degli angoli (semplificato)
    ddphi = tau_phi;
    ddtheta = tau_theta;
    ddpsi = tau_psi;
    
    % Output delle derivate
    dpdt = [dx; dy; dz; dphi; dtheta; dpsi; ddx; ddy; ddz; ddphi; ddtheta; ddpsi];
end