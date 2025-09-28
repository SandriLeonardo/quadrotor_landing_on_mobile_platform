% DEBUG WORKSPACE - Find what's happening with 'out' variable

fprintf('=== WORKSPACE DEBUG ===\n');

% Check if 'out' exists
if exist('out', 'var')
    fprintf('✓ Variable "out" exists\n');
    
    % Check its structure
    fprintf('Type of out: %s\n', class(out));
    
    if isstruct(out)
        fields = fieldnames(out);
        fprintf('Fields in out: %s\n', strjoin(fields, ', '));
        
        % Check quadrotor_states specifically
        if isfield(out, 'quadrotor_states')
            fprintf('✓ quadrotor_states field exists\n');
            
            % Check structure
            qs = out.quadrotor_states;
            fprintf('Type of quadrotor_states: %s\n', class(qs));
            
            if isstruct(qs)
                qs_fields = fieldnames(qs);
                fprintf('Fields in quadrotor_states: %s\n', strjoin(qs_fields, ', '));
                
                % Check signals
                if isfield(qs, 'signals')
                    signals = qs.signals;
                    fprintf('Type of signals: %s\n', class(signals));
                    
                    if isstruct(signals) && isfield(signals, 'values')
                        values = signals.values;
                        fprintf('Values size: %s\n', mat2str(size(values)));
                        fprintf('Values type: %s\n', class(values));
                        
                        if ~isempty(values)
                            fprintf('First few values:\n');
                            disp(values(1:min(3,size(values,1)), :));
                        end
                    end
                end
                
                % Check time
                if isfield(qs, 'time')
                    time_data = qs.time;
                    fprintf('Time size: %s\n', mat2str(size(time_data)));
                    fprintf('Time range: %.6f to %.6f\n', time_data(1), time_data(end));
                end
            end
        else
            fprintf('❌ No quadrotor_states field found\n');
        end
    end
else
    fprintf('❌ Variable "out" does not exist\n');
end

% Check workspace for other similar variables
fprintf('\n--- OTHER VARIABLES IN WORKSPACE ---\n');
vars = who;
matching_vars = vars(contains(vars, 'out', 'IgnoreCase', true));
if ~isempty(matching_vars)
    fprintf('Variables containing "out": %s\n', strjoin(matching_vars, ', '));
end

fprintf('\n=== END WORKSPACE DEBUG ===\n');