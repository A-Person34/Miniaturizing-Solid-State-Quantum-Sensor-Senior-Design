clear
clc

port = "COM4";
baud = 115200;

numSweeps = 10;

allData = [];

for sweep = 1:numSweeps
    
    fprintf("Starting Sweep %d\n", sweep)
    
    s = serialport(port,baud);
    configureTerminator(s,"LF");
    s.Timeout = 20;
    flush(s);
    
    pause(2)
    
    data = [];
    
    % ===== CREATE FIGURE FOR THIS SWEEP =====
    figure(sweep)
    h = animatedline('LineWidth',2);
    
    xlabel("Frequency (MHz)")
    ylabel("Voltage (V)")
    title("Microwave Sweep " + sweep)
    grid on
    
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
                    
                    addpoints(h,f,v)
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
    
    % ===== STORE DATA =====
    
    if isempty(allData)
        freq = data(:,1);
        allData = zeros(length(freq),numSweeps);
    end
    
    allData(:,sweep) = data(:,2);
    
end

disp("All sweeps complete")

% ===== AVERAGE GRAPH =====

avgSignal = mean(allData,2);

figure(11)

plot(freq,avgSignal,'LineWidth',3)

xlabel("Frequency (MHz)")
ylabel("Voltage (V)")
title("Average of 10 Sweeps")

grid on


% ===== INTEGRAL GRAPH =====

integralSignal = cumtrapz(freq,avgSignal);

figure(12)

plot(freq,integralSignal,'LineWidth',3)

xlabel("Frequency (MHz)")
ylabel("Integral")

title("Integral of Average Signal")

grid on