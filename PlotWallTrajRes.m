clear all;
syms t;
% T = 2;


gray = '#a1a3a6';


red_color = '#a7324a';
green_color = '#2b6447';
blue_color = '#145b7d';
purple_color = '#472d56';
filler = '#f6f5ec';

curve_width = 1.2;
curve_width_ref = 0.8;

load('DESKTOP-4J3FS1P_2023_10_01_14_47_27UFlapperInMocapgoodline.mat');
StartSec = 3;
Find_l_StartSec = find(record_time_stamp > StartSec);
Front = Find_l_StartSec(1);

EndSec   = 20;
Find_l_EndSec = find(record_time_stamp > EndSec);
Rear = Find_l_EndSec(1);

Cut = Front : Rear;

figure;
subplot(3,1,1); 
hold on;
% p1=plot(time_secs, x_pos,'--', 'Linewidth', curve_width_ref, 'color', red_color);
p3=plot(record_time_stamp(Cut)-StartSec,record_p(Cut,1)-record_p(Front,1),...
    'Linewidth', curve_width, 'color', red_color);
xlim([0, EndSec - StartSec]);
% ylim([-1,1]);
% legend([p1,p3],'Reference X pos.','Real X pos.');

subplot(3,1,2);
hold on;
% p1 = plot(time_secs, y_pos,'--', 'Linewidth', curve_width_ref, 'color', green_color);
p3 =plot(record_time_stamp(Cut)-StartSec,record_p(Cut,2)-record_p(Front,2),...
    'Linewidth', curve_width, 'color', green_color);
xlim([0, EndSec - StartSec]);
% legend([p1,p3],'Reference Y pos.','Real Y pos.');

subplot(3,1,3);
hold on;
% p1=plot(time_secs, z_pos,'--', 'Linewidth', curve_width_ref, 'color', blue_color);
p3=plot(record_time_stamp(Cut)-StartSec,record_p(Cut,3)-record_p(Front,3),...
    'Linewidth', curve_width, 'color', blue_color);
xlim([0, EndSec - StartSec]);
% ylim([-0.3,.3]);
% legend([p1,p3],'Reference Z pos.','Real Z pos.');

set(gcf, 'Position', [100 100 600 450]); 

