% mindstorm 매트랩 매뉴얼 참고 
% https://kr.mathworks.com/hardware-support/lego-mindstorms-ev3-matlab.html

% port info
% A B C D => B(motor1), C(motor2)
% -------
% 1 2 3 4

%% clear all
clc;clear;close all
clear mylego
fclose('all')

%% connect through bluetooth
clear mylego

mylego = legoev3('bt','00165354475A')

%% motor config
% Change based on your motor port numbers
mymotor1 = motor(mylego, 'B');              % Set up motor
mymotor2 = motor(mylego, 'C');

%%
% Application parameters
EXE_TIME = 8;                              % Application running time in seconds
PERIOD = 0.1;                               % Sampling period
SPEED = 60;                                 % Motor speed
OFFSET = 0;
phi = 0;
P = 0.01;                                   % P controller parameter
%-------------------------------------------

resetRotation(mymotor1);                    % Reset motor rotation counter
resetRotation(mymotor2);

start(mymotor1);                            % Start motor
start(mymotor2);

t = timer('TimerFcn', 'stat=false;', 'StartDelay',EXE_TIME);
start(t);

% t2 = timer('TimerFcn', 'stat2=false;', 'StartDelay',EXE_TIME/2);
% start(t2);

%% Operations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stat = true;
% stat2 = true;
lastR1 = 0;
lastR2 = 0;

q_robot = [0,0,0]; % [x,y,phi] : xy 위치 및 방위각
q_des = [-1,1];

Kp = 1;
Kd = 0;
Ki = 0;

phi_des_prev   = 0;
phi_robot_prev = 0;
e_phi = 0;

feedforward = 0;

while stat == true                          % Quit when times up
%     if stat2 == false
%         q_des = [1 1];
%     end
    
    phi_des = -asin((q_des(1)-q_robot(1))/norm(q_des-q_robot(1:2)))*180/pi;
    phi_robot = q_robot(3);
    
    phi_des_dot = (phi_des - phi_des_prev) / PERIOD;
    phi_robot_dot = (phi_robot - phi_robot_prev) / PERIOD;
    
    e_phi = phi_des - phi_robot;
    e_phi_dot = phi_des_dot - phi_robot_dot;
    e_phi_int = e_phi + e_phi_dot*PERIOD;
    
    phi_des_prev = phi_des;
    phi_robot_prev = phi_robot;
    
    OFFSET = feedforward + e_phi*Kp + e_phi_dot*Kd + e_phi_int*Ki;
    
    % 밑단 모터 컨트롤 부분 (인풋 : SPEED, OFFSET)
    mymotor1.Speed = SPEED - OFFSET;                     % Set motor speed
    mymotor2.Speed = SPEED + OFFSET;

    r1 = readRotation(mymotor1);            % Read rotation counter in degrees
    r2 = readRotation(mymotor2);  
    
    speed1 = (r1 - lastR1)/PERIOD;          % Calculate the real speed in d/s
    speed2 = (r2 - lastR2)/PERIOD;
    
    diff = speed1 - speed2 + 2*OFFSET;                 % P controller
    mymotor1.Speed = mymotor1.Speed - int8(diff * P);
    
    lastR1 = r1;
    lastR2 = r2;
    
    pause(PERIOD);                          % Wait for next sampling period
end

%% Clean up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stop(mymotor1);                             % Stop motor 
stop(mymotor2);

clear
