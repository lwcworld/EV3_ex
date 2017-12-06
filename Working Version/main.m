% mindstorm ��Ʈ�� �Ŵ��� ���
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

%mylego = legoev3('bt ','00165355d61e')
mylego = legoev3('usb')

% beep 2 times
beep(mylego); pause(0.5); beep(mylego)

%% motor config
% Change based on your motor port numbers
mymotor1 = motor(mylego, 'B'); % Set up motor
mymotor2 = motor(mylego, 'C'); 

%% path generation
while true
    fp = fopen('/home/lwc/git/EV3_ex/RobotDetect/data.txt');
    temp = fscanf(fp, '%f', [1 inf]);
    fclose(fp);
    if ~isempty(temp)
        break;
    end
end
pos = temp;
x = (pos(1) + pos(3))/2;
y = (pos(2) + pos(4))/2;
yaw = mod(pos(5), 360);        % yaw angle in [0, 360]
if yaw > 180
    yaw = yaw - 360; % yaw angle in [-180, 180]
    yaw = yaw*pi/180;
end

p1 = [x, y, yaw]'; % robot's initial [x,y,heading]
p2 = [450,450,55*pi/180]'; % [x, y, heading] = [mm, mm , rad]
r_min = 100; % [mm]
stepsize = 50; % [mm]
flag_path = 1;
[cost, path] = dubins_curve(p1, p2, r_min, stepsize, flag_path);

% path : [x, y, heading]

figure(1); hold on; grid on;
plot(path(:,1), path(:,2))

%%
% Application parameters
EXE_TIME = 100;                             % Application running time in seconds
% PERIOD = 0.1;                             % Sampling period
SPEED = 60;                                 % Motor speed
OFFSET = 0;                                 % Heading Control input
phi = 0;
P = 0.01;                                   % P controller parameter
%-------------------------------------------

resetRotation(mymotor1);                    % Reset motor rotation counter
resetRotation(mymotor2);

start(mymotor1);                            % Start motor
start(mymotor2);

t = timer('TimerFcn', 'stat=false;', 'StartDelay',EXE_TIME);
start(t);

%% Operations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stat = true;
% stat2 = true;
lastR1 = 0;
lastR2 = 0;

i_des = 1;

q_robot = [0,0,0]; % [x,y,phi] : xy ��ġ �� ������ % �� time ������Ʈ �ǰ� ¥����
q_des = path(1,1:2);

Kp = 200;
Kd = 0;
Ki = 0;

yaw_des_prev   = 0;
yaw_robot_prev = 0;
e_phi = 0;

feedforward = 0;

fp = fopen('/home/lwc/git/EV3_ex/RobotDetect/data.txt');
temp = fscanf(fp, '%f', [1 inf]);
fclose(fp);
pos = temp;

flag_start = 0;

while true                          % Quit when times up
    fp = fopen('/home/lwc/git/EV3_ex/RobotDetect/data.txt');
    temp = fscanf(fp, '%f', [1 inf]);
    fclose(fp);
    
    if isequal(temp, pos) || isempty(temp)
        % Give some amount of delay and continue
        pause(0.01);
        continue;
    else
        
        if (flag_start == 0)
            PERIOD = 0.05;    % to avoid dividing by 0
            tic
            time_total = toc;
        else
            time_total_ex = time_total;
            time_total = toc
            PERIOD = time_total - time_total_ex;
        end
        
        pos = temp;
        if pos > 1e5
            continue;
        end
        x = (pos(1) + pos(3))/2;
        y = (pos(2) + pos(4))/2;
        yaw = mod(pos(5), 360);        % yaw angle in [0, 360]
        if yaw > 180
            yaw = yaw - 360; % yaw angle in [-180, 180]
        end
        yaw = yaw*pi/180;
        
        q_robot = [x,y,yaw];
        
        if norm(q_robot(1,1:2) - q_des) <= 26 % ��ǥ���� �Ÿ��� 10cm(0.1m) �����ΰ��
            i_des = i_des + 1;
            if i_des > length(path)
                break;
            end
            q_des = path(i_des, 1:2);
        else
            SPEED = 100;
        end
        
        yaw_des = atan2(q_des(2)-q_robot(2), q_des(1)-q_robot(1));
        yaw_robot = q_robot(3);
        
        yaw_des_dot = (yaw_des - yaw_des_prev) / PERIOD;
        yaw_robot_dot = (yaw_robot - yaw_robot_prev) / PERIOD;
        
        e_phi = yaw_des - yaw_robot;
        e_phi_dot = yaw_des_dot - yaw_robot_dot;
        e_phi_int = e_phi + e_phi_dot*PERIOD;
        
        yaw_des_prev = yaw_des;
        yaw_robot_prev = yaw_robot;
        
        OFFSET = feedforward + e_phi*Kp + e_phi_dot*Kd + e_phi_int*Ki;
        
        % motor control part (input : SPEED, OFFSET)
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
        
        figure(1)
        plot(q_robot(1), q_robot(2), '.r');
        plot(q_des(1), q_des(2), 'ob');
        axis equal
        [q_des, q_robot(1:2), yaw_des*180/pi, yaw_robot*180/pi]
        
        pause(0.1);                          % Wait for next sampling period
    end
end

%% Clean up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stop(mymotor1);                             % Stop motor
stop(mymotor2);
% clear