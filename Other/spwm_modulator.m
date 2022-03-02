%% Modulator to generate SinePWM signals
% A signal in this case a sine is modulated using a triangle wave (carrier)
% to generate a PWM signal that can be implemented on a MCU

%%
clear all
close all
clc

%% generate carrier wave (triangle)
periods = 10;

lowlim = -20;
uplim = 20;
stepsize = 1;

sig = [[lowlim:stepsize:uplim],[uplim-stepsize:-stepsize:lowlim]];


sig = [sig,sig]
plot(sig)


