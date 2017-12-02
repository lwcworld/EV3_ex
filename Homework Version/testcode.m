% reference
% https://kr.mathworks.com/hardware-support/lego-mindstorms-ev3-matlab.html

% port info
% A B C D
% -------
% 1 2 3 4


%% clear all
clc;clear;close all
clear mylego
fclose('all')

%% connect by bluetooth
% bluetooth connection
% mylego = legoev3('bluetooth','COM5')
mylego = legoev3('bt','00165354475A')

% beep 2 times 
beep(mylego); pause(0.5); beep(mylego)

%% Clear the LCD, and then write text on row 2, column 3.
clearLCD(mylego)
writeLCD(mylego,'Hello, LEGO!',2,3)

%% Play a 500 Hz tone on the speaker for 3 seconds, with the volume level set to 20.
playTone(mylego,500,3,20)

%% Read the status of the buttons. If the button is pressed, the status is 1. Otherwise, the status is 0.
readButton(mylego,'up')
readButton(mylego,'down')
readButton(mylego,'right')
readButton(mylego,'left')
readButton(mylego,'center')

%% Illuminate the status light with a red LED, and then turn it off.
writeStatusLight(mylego,'red')
writeStatusLight(mylego,'off')

%% config motor
leftmotor = motor(mylego,'B')
rightmotor = motor(mylego, 'C')

leftmotor.Speed = 20
rightmotor.Speed = 20
start(leftmotor)
start(rightmotor)

pause(1)
stop(leftmotor)
stop(rightmotor)

%% open loop control 
edit('gostraight_openloop.m')


%%
edit('gostraight_closeloop.m')





