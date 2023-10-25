clear all;
syms t;
% T = 2;
load('wall_obstraction.mat')





pos_x_res(t)   = poly2sym(coef(1:7), t);
pos_y_res(t)   = poly2sym(coef(8:14), t);
pos_z_res(t)   = poly2sym(coef(15:21), t);

pos_x_res1(t)   = poly2sym(coef((1:7)+21), t);
pos_y_res1(t)   = poly2sym(coef((8:14)+21), t);
pos_z_res1(t)   = poly2sym(coef((15:21)+21), t);
% psi_res(t)   = poly2sym(coef(22:28), t);

vel_x_res = diff(pos_x_res,t,1);
vel_y_res = diff(pos_y_res,t,1);
vel_z_res = diff(pos_z_res,t,1);

pos_psi_res(t) = atan2(vel_y_res(t),vel_x_res(t));
vel_psi_res(t) = diff(pos_psi_res,t,1);

time_secs = 0.01 : 0.01 : T;
x_pos = double(pos_x_res(time_secs));
y_pos = double(pos_y_res(time_secs));
z_pos = double(pos_z_res(time_secs));

x_pos1 = double(pos_x_res1(time_secs));
y_pos1 = double(pos_y_res1(time_secs));
z_pos1 = double(pos_z_res1(time_secs));

psi = double(pos_psi_res(time_secs));

x_vel = double(vel_x_res(time_secs));
y_vel = double(vel_y_res(time_secs));
z_vel = double(vel_z_res(time_secs));
psi_vel = double(vel_psi_res(time_secs));

gray = '#a1a3a6';


red_color = '#a7324a';
green_color = '#2b6447';
blue_color = '#145b7d';
purple_color = '#472d56';
filler = '#f6f5ec';

curve_width = 1.2;
curve_width_ref = 0.8;

load('DESKTOP-4J3FS1P_2023_10_02_08_27_14UFlapperInMocapwall3.mat');
StartSec = 3;
Find_l_StartSec = find(record_time_stamp > StartSec);
Front = Find_l_StartSec(1);

EndSec   = 7;
Find_l_EndSec = find(record_time_stamp > EndSec);
Rear = Find_l_EndSec(1);

Cut = Front : Rear;

figure;
subplot(3,1,1); 
hold on;
plot(time_secs, x_pos,'--', 'Linewidth', curve_width_ref, 'color', red_color);
plot(time_secs + T, x_pos1,'--', 'Linewidth', curve_width_ref, 'color', red_color);

plot(record_time_stamp(Cut)-StartSec,record_p(Cut,1)-record_p(Front,1),...
    'Linewidth', curve_width, 'color', red_color);
xlim([0, 4]);
ylim([-1, 1]);
legend('Reference X pos.','Real X pos.');

subplot(3,1,2);
hold on;
plot(time_secs, y_pos, '--', 'Linewidth', curve_width_ref, 'color', green_color);
plot(record_time_stamp(Cut)-StartSec,record_p(Cut,2)-record_p(Front,2),...
    'Linewidth', curve_width, 'color', green_color);
plot(time_secs + T, y_pos1, '--', 'Linewidth', curve_width_ref, 'color', green_color);


xlim([0, 4]);
legend('Reference Y pos.','Real Y pos.');

subplot(3,1,3);
hold on;
plot(time_secs, z_pos, '--', 'Linewidth', curve_width_ref, 'color', blue_color);
plot(record_time_stamp(Cut)-StartSec,record_p(Cut,3)-record_p(Front,3),...
    'Linewidth', curve_width, 'color', blue_color);
plot(time_secs+T, z_pos1, '--', 'Linewidth', curve_width_ref, 'color', blue_color);


xlim([0, 4]);
legend('Reference Z pos.','Real Z pos.');

set(gcf, 'Position', [100 100 600 450]); 

TheCut = record_time_stamp(Cut)-StartSec;
TheSplit =  find(TheCut > T);
TheSplit = TheSplit(1);

X_ref_0 = double(pos_x_res(TheCut(1:TheSplit-1)))';
X_ref_1 = double(pos_x_res1(TheCut(TheSplit:end)-T))';
X_ref = [X_ref_0; X_ref_1];

Y_ref_0 = double(pos_y_res(TheCut(1:TheSplit-1)))';
Y_ref_1 = double(pos_y_res1(TheCut(TheSplit:end)-T))';
Y_ref = [Y_ref_0; Y_ref_1];

Z_ref_0 = double(pos_z_res(TheCut(1:TheSplit-1)))';
Z_ref_1 = double(pos_z_res1(TheCut(TheSplit:end)-T))';
Z_ref = [Z_ref_0; Z_ref_1];

X_error = double(X_ref - (record_p(Cut,1)-record_p(Front,1)));
Y_error = double(Y_ref - (record_p(Cut,2)-record_p(Front,2)));
Z_error = double(Z_ref - (record_p(Cut,3)-record_p(Front,3)));

disp('Along track error MAX'+string(max(abs(Y_error))) );
disp('Along track error RMS'+string(rms(abs(Y_error))) );

disp('Cross track error MAX'+string(max(abs(X_error))) );
disp('Cross track error RMS'+string(rms(abs(X_error))) );

disp('Altitude error MAX'+string(max(abs(Z_error))) );
disp('Altitude error RMS'+string(rms(abs(Z_error))) );        