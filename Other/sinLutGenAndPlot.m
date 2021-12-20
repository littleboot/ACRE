close all;
clear;
clc;

%config: Sine lookup table
lutSize = 128;
lutMaxVal = 255;
lutMinVal = 0;
pwmMaxVal = 255;

%generate Lookup tables
sinLut = getLut(lutSize);
sinLutMapped = round(mapRange( sinLut, -1, 1, lutMinVal, lutMaxVal));

%generate sampled SPWM signal fro plotting, using software PWM Timer
AAR = pwmMaxVal;
CNT = 0;
REP_CNT = lutSize;

LUT_POS = 1;
signal = [];
for i = [1:AAR * REP_CNT]
    if CNT == AAR
        CNT = 0;
        LUT_POS = LUT_POS + 1;
    end
    
    if CNT <= sinLutMapped(LUT_POS)
        signal = [signal, 1];
    else
        signal = [signal, 0];
    end
    CNT = CNT + 1;
end

%Plot figures
% bar(sinLut)
figure
tiledlayout(2,1);
nexttile
% bar(signal)
plot(upsample(signal, 1000))

b = nexttile;
bar(sinLutMapped)
set(b,'XLim',[1 numel(sinLutMapped)])

function lut = getLut(lutSize)
    for i = [1:1:lutSize]
        lut(i) = sin( 2 * pi * i / lutSize );
    end
end

function out = mapRange(x, in_min, in_max, out_min, out_max )
    if x <= in_min
        out = in_min;
    elseif x >= in_max
        out = in_max;
    else
        out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    end
end

%Sample the generated signals at a higher frequancy, so the line plot
%transitions of the square wave look good
%https://nl.mathworks.com/matlabcentral/answers/369002-how-to-increase-elements-of-a-vector-without-changing-its-plot
function out = upsample(X, newSize)
    out = interp1(1:length(X), X, linspace(1, length(X), newSize), 'nearest');
end

function displayAsArray(x)
    
end
