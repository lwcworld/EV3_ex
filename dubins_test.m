% p1 = [x, y, degree1]';
% p2 = [x, y, degree2]';

p1 = [0,0,0]';
p2 = [0,1000,0]'; % 1000mm 
r_min = 1000;
stepsize = 100; % 1000mm
flag_path = 1;
[cost, path] = dubins_curve(p1, p2, r_min, stepsize, flag_path);