p0 = [0, 0, 0];            % Punto iniziale [x0, y0, z0]
pf = [10, 5, 2];           % Punto finale [xf, yf, zf]
t_total = 10;              % Tempo totale (secondi)
t = 0:0.01:t_total;        % Vettore tempo con campionamento di 0.01s

% Genera la traiettoria (puoi scegliere tra 'constant_vel', 'trapezoidal', 'polynomial')
[pos, vel, acc] = generate_linear_trajectory(p0, pf, t, t_total, 'polynomial');