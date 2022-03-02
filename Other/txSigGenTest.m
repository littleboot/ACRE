close all;
clear;
clc;

%config: Sine lookup table
lutSize = 32;
lutMaxVal = 255;
lutMinVal = 0;
pwmMaxVal = 255;

%generate Lookup tables
sinLut = getLut(lutSize);
sinLutMapped = round(mapRange( sinLut, -1, 1, lutMinVal, lutMaxVal));

evenSinLut = getHalfLut(16);
oddSinLut = circshift(evenSinLut, 16/2);

%test, test addition of positive half of sine and cosine
%evenSinLut = evenSinLut + oddSinLut;
%oddSinLut = evenSinLut;
%

evenSinPwmLut = getPwmSig(evenSinLut);
oddSinPwmLut = getPwmSig(oddSinLut);

evenOddSinComb = combineAlternating(evenSinLut, oddSinLut);

%new
x = [evenSinLut, -oddSinLut];
%x = combineAlternating(x,x);
x_pwm = getPwmSig(x);

% Test 2
n = getLutAmp(16, 8); % get sin lut of 32 samples with amplitude of 16 max
n = round(n); % Round to integer

figure;
title('test rounded preamplified LUT');
tiledlayout(2,2);
nexttile;
plot(n);
nexttile;
bar(n);


%
figure;
tiledlayout(2,1);
nexttile;
bar(x);
nexttile;
%plot(x_pwm);
bar(x_pwm);

% Create figure..........................
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');
% Create plot
plot(x_pwm);
% X/Y-limits of the axes
xlim(axes1,[0 length(x_pwm)]);
ylim(axes1,[0 12]);
box(axes1,'on');
hold(axes1,'off');



%generate sampled SPWM signal for plotting, using software PWM Timer
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
%plot(upsample(signal, 1000))
plot(signal)

b = nexttile;
bar(sinLutMapped)
set(b,'XLim',[1 numel(sinLutMapped)])

% even odd sampes plot
figure
tiledlayout(4,1);
nexttile;
bar(evenSinLut);
nexttile;
plot(evenSinPwmLut);
nexttile;
bar(oddSinLut);
nexttile;
plot(oddSinPwmLut);

figure;
tiledlayout(2,1);
nexttile;
bar(evenOddSinComb);
nexttile;
plot(getPwmSig(evenOddSinComb));

function lut = getLut(lutSize)
    for i = [1:1:lutSize]
        lut(i) = sin( 2 * pi * i / lutSize );
    end
end

function lut = getLutAmp(lutSize, amp)
    for i = [1:1:lutSize]
        lut(i) = amp * sin( 2 * pi * i / lutSize );
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

% Only keep positive samples. aka half lut is set to 0
function lut = getHalfLut(lutSize)
    for i = [1:1:lutSize]
        lut(i) = sin( 2 * pi * i / lutSize );
        if lut(i) < 0
            lut(i) = 0;
        end
    end
end


function pwmSig = getPwmSig(sig)
    AAR = 16;
    CNT = 0;
    REP_CNT = length(sig);
    
    sigMap = mapRange( sig, -1, 1, 0, AAR);

    LUT_POS = 1;
    pwmSig = [];
    for i = [1:AAR * REP_CNT]
        if CNT == AAR
            CNT = 0;
            LUT_POS = LUT_POS + 1;
        end
        
        if CNT <= sigMap(LUT_POS)
            pwmSig = [pwmSig, 1];
        else
            pwmSig = [pwmSig, 0];
        end
        CNT = CNT + 1;
    end
end

% Combine 2 arrays into one, alternate trough samples
function combined = combineAlternating(arr1, arr2)
    combined = [];
    
    iarr1 = 1;
    iarr2 = 1;
    
    for i = [1:length(arr1)*2]
        if mod(i,2)
            combined(i) = arr1(iarr1);
            iarr1 = iarr1 + 1;
        else
            combined(i) = arr2(iarr2);
            iarr2 = iarr2 + 1;
        end
    end
end
    
