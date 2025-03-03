function [pos, vel, acc] = generate_linear_trajectory(p0, pf, t, t_total, profile_type)
    % Genera una traiettoria rettilinea da p0 a pf con profili di posizione, velocità
    % e accelerazione per ciascuna componente x, y, z
    %
    % Parametri:
    % p0 - punto iniziale [x0, y0, z0]
    % pf - punto finale [xf, yf, zf]
    % t - vettore tempo
    % t_total - tempo totale per completare la traiettoria
    % profile_type - tipo di profilo: 'constant_vel', 'trapezoidal', 'polynomial'
    %
    % Outputs:
    % pos - matrice Nx3 con le posizioni [x, y, z] a ogni istante
    % vel - matrice Nx3 con le velocità [vx, vy, vz] a ogni istante
    % acc - matrice Nx3 con le accelerazioni [ax, ay, az] a ogni istante
    
    if nargin < 5
        profile_type = 'polynomial'; % Default: profilo polinomiale
    end
    
    N = length(t);
    pos = zeros(N, 3);
    vel = zeros(N, 3);
    acc = zeros(N, 3);
    
    % Per ogni dimensione (x, y, z)
    for dim = 1:3
        p_start = p0(dim);
        p_end = pf(dim);
        
        switch profile_type
            case 'constant_vel'
                % Profilo a velocità costante
                v_const = (p_end - p_start) / t_total;
                
                pos(:, dim) = p_start + v_const * t;
                vel(:, dim) = v_const * ones(size(t));
                acc(:, dim) = zeros(size(t));
                
            case 'trapezoidal'
                % Profilo trapezoidale in velocità
                % Usa 20% del tempo per accelerazione e 20% per decelerazione
                t_acc = 0.2 * t_total;
                t_dec = 0.2 * t_total;
                t_const = t_total - t_acc - t_dec;
                
                % Calcola velocità massima
                v_max = (p_end - p_start) / (t_const + (t_acc + t_dec)/2);
                
                % Calcola accelerazione
                a_acc = v_max / t_acc;
                a_dec = -v_max / t_dec;
                
                for i = 1:N
                    ti = t(i);
                    
                    if ti <= t_acc
                        % Fase di accelerazione
                        pos(i, dim) = p_start + 0.5 * a_acc * ti^2;
                        vel(i, dim) = a_acc * ti;
                        acc(i, dim) = a_acc;
                        
                    elseif ti <= t_acc + t_const
                        % Fase a velocità costante
                        pos(i, dim) = p_start + 0.5 * a_acc * t_acc^2 + v_max * (ti - t_acc);
                        vel(i, dim) = v_max;
                        acc(i, dim) = 0;
                        
                    elseif ti <= t_total
                        % Fase di decelerazione
                        t_dec_start = t_acc + t_const;
                        t_in_dec = ti - t_dec_start;
                        
                        pos(i, dim) = p_start + 0.5 * a_acc * t_acc^2 + v_max * t_const + 
                                     v_max * t_in_dec + 0.5 * a_dec * t_in_dec^2;
                        vel(i, dim) = v_max + a_dec * t_in_dec;
                        acc(i, dim) = a_dec;
                        
                    else
                        % Dopo la fine della traiettoria
                        pos(i, dim) = p_end;
                        vel(i, dim) = 0;
                        acc(i, dim) = 0;
                    end
                end
                
            case 'polynomial'
                % Profilo polinomiale (quinto ordine)
                for i = 1:N
                    % Normalizza il tempo tra 0 e 1
                    s = min(1, max(0, t(i) / t_total));
                    
                    % Polinomio del quinto ordine (garantisce velocità e accelerazione zero agli estremi)
                    pos(i, dim) = p_start + (p_end - p_start) * (10*s^3 - 15*s^4 + 6*s^5);
                    
                    % Derivate
                    vel(i, dim) = (p_end - p_start) * (30*s^2 - 60*s^3 + 30*s^4) / t_total;
                    acc(i, dim) = (p_end - p_start) * (60*s - 180*s^2 + 120*s^3) / (t_total^2);
                end
        end
    end
end

