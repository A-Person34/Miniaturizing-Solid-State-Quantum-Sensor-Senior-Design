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
        lastDataTime = tic;   % reset timer when data arrives
        
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
        % If no data for 2 seconds → sweep done
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

% Optional: Final clean plot redraw
figure
plot(data(:,1), data(:,2), 'LineWidth',2)
xlabel("Frequency (MHz)")
ylabel("Voltage (V)")
title("Microwave Sweep (Final)")
grid on