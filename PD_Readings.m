clear
clc

port = "COM4";   % <-- CHANGE THIS
baud = 115200;


s = serialport(port, baud);
configureTerminator(s,"LF");
s.Timeout = 20;
flush(s);

disp("Press RESET on the ESP32 now...");
pause;

data = [];

figure
h = animatedline('LineWidth',2);
xlabel("Frequency (MHz)")
ylabel("Voltage (V)")
title("Microwave Sweep (Live)")
grid on

disp("Reading sweep...")

lastDataTime = tic;

while true
    
    if s.NumBytesAvailable > 0
        
        line = readline(s);
        lastDataTime = tic;
        
        if contains(line,"Frequency")
            continue
        end
        
        vals = split(line,",");
        
        if length(vals) == 2
            f = str2double(vals(1));
            v = str2double(vals(2));
            
            if ~isnan(f) && ~isnan(v)
                data = [data; f v];
                
                addpoints(h, f, v);
                drawnow limitrate
            end
        end
        
    else
        if toc(lastDataTime) > 2
            break
        end
    end
end

clear s

if isempty(data)
    error("No data received.")
end

disp("Sweep complete.")

% ===== FINAL VOLTAGE FIGURE =====
figure
plot(data(:,1), data(:,2), 'LineWidth',2)
xlabel("Frequency (MHz)")
ylabel("Voltage (V)")
title("Microwave Sweep (Final)")
grid on

% ===== FINAL INTEGRAL FIGURE =====
if size(data,1) > 2
    
    freq = data(:,1);
    volt = data(:,2);
    
    % Numerical integral using trapezoidal rule
    integral_signal = cumtrapz(freq, volt);
    
    figure
    plot(freq, integral_signal, 'LineWidth',2)
    xlabel("Frequency (MHz)")
    ylabel("Integral of Voltage")
    title("Antiderivative / Integral of Signal")
    grid on
    
end
