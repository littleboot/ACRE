% close all
clc
clear

%signal props
f = 153.6E3; %hz
T = 1/f;

%Upsampled signal sample size
upsamplesize = 512*10;

% Pulse width measurements in 4T 
% see" Signal sample calculation.xlsx" measured with logic analyser
% 128 positions in LUT, minimum
pulses4T = [5 1 3 1 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 3 1 5 1 3 1 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 3 1];
% 512 Positions in LUT
pulses1T = pulses4T*4;

%Create LUT from pulse width array, can be imported on MCU as array to
%output the TX signals
pol = 1;
x = [];
for i = pulses1T
    for j = 1:i
        if pol == 1
            x = [x, 1];
        else
            x = [x, 0];
        end
    end
    
    if pol == 1
        pol = 0;
    else
        pol = 1;
    end
    
end

% TX signals are shifted 45 degree in phase
TX1 = x;
TX2 = circshift(x, length(x) * 1 / 8);
TX3 = circshift(x, length(x) * 2 / 8);
TX4 = circshift(x, length(x) * 3 / 8);
TX5 = circshift(x, length(x) * 4 / 8);
TX6 = circshift(x, length(x) * 5 / 8);
TX7 = circshift(x, length(x) * 6 / 8);
TX8 = circshift(x, length(x) * 7 / 8);

% figure('Name','TX signals');
% fig = tiledlayout(8, 1);
% 
% nexttile
% bar(TX1)
% axis off
% nexttile
% bar(TX2)
% axis off
% nexttile
% bar(TX3)
% axis off
% nexttile
% bar(TX4)
% axis off
% nexttile
% bar(TX5)
% axis off
% nexttile
% bar(TX6)
% axis off
% nexttile
% bar(TX7)
% axis off
% nexttile
% bar(TX8)
% axis off
% 
% fig.Padding = 'compact';
% fig.TileSpacing = 'compact';

%Plot line graphs: 
%To make the pwm transition look good in a line plot the
%signal is sampled at a higher frequency.

%upsample the TX signals before plotting
TX1_U = upsample(TX1, upsamplesize);
TX2_U = upsample(TX2, upsamplesize);
TX3_U = upsample(TX3, upsamplesize);
TX4_U = upsample(TX4, upsamplesize);
TX5_U = upsample(TX5, upsamplesize);
TX6_U = upsample(TX6, upsamplesize);
TX7_U = upsample(TX7, upsamplesize);
TX8_U = upsample(TX8, upsamplesize);

% figure('Name','TX signals line graphs');
fig = tiledlayout(11, 1);
fig.Padding = 'compact';
fig.TileSpacing = 'none';

nexttile;
plot( TX1_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX2_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX3_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX4_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX5_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX6_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX7_U )
axis off
xlim([0 upsamplesize])
nexttile;
plot( TX8_U )
axis off
xlim([0 upsamplesize])

% ADD 4 of the TX signals together and plot the result in next tile
TX1_UtoTX4_U = TX1_U + TX2_U + TX3_U + TX4_U;
% TX1_UtoTX4_U = TX1_U .* TX2_U .* TX3_U .* TX4_U;

f = nexttile;
plot(TX1_UtoTX4_U)
axis off
set(f,'XLim',[1 upsamplesize])

% Plot bias signal
BIAS = generateBias();
BIAS = circshift(BIAS, 0); %Shift can be used to align signals
BIAS_U = upsample(BIAS, upsamplesize);

f = nexttile;
plot( BIAS_U )
axis off
set(f,'XLim',[1 upsamplesize])

%plot what is left on receiver when biad is appllied
RX_SIG_U = TX1_UtoTX4_U .* BIAS_U;

f = nexttile;
plot( RX_SIG_U )
axis off
set(f,'XLim',[1 upsamplesize])


%Test bias alignment
% figure
% plot(TX1_U)
% xlim([0 upsamplesize])
% hold on
% plot(BIAS_U*0.9)
% xlim([0 upsamplesize])

printBuffer(TX1)
printBuffer(BIAS)

%Sample the generated signals at a higher frequancy, so the line plot
%transitions of the square wave look good
%https://nl.mathworks.com/matlabcentral/answers/369002-how-to-increase-elements-of-a-vector-without-changing-its-plot
function out = upsample(X, newSize)
    out = interp1(1:length(X), X, linspace(1, length(X), newSize), 'nearest');
end

function bias = generateBias
    T = [0,0,0,1,1,1,1,1];
%     T = [1,1,1,0,0,0,0,0];
%     T = [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0];
    bias = [];
    for i = [1:512/numel(T)]
       bias =  [bias, T];
    end
end

function printBuffer(buff)
    fprintf("uint8_t Buff[%i] = {%i, ",numel(buff), buff(1))
    for i = [2:numel(buff)-1]
        fprintf("%i, ",buff(i))
    end
    fprintf("%i};\n", buff(numel(buff)) )
end