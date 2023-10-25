clear all;
figure; 
Threename = {'DESKTOP-4J3FS1P_2023_10_01_14_47_27UFlapperInMocapgoodline.mat',...
    'DESKTOP-4J3FS1P_2023_10_01_14_56_42UFlapperInMocapvideoline.mat',...
    'DESKTOP-4J3FS1P_2023_10_02_06_59_30UFlapperInMocap.mat'};
% load('DESKTOP-4J3FS1P_2023_10_12_13_07_47UFlapperInMocap.mat')
% DESKTOP-4J3FS1P_2023_10_02_06_59_30UFlapperInMocap.mat
gray = '#a1a3a6';


red_color = [178 34 34]./255;
green_color = [	0 139 0]./255;
blue_color = [39 64 139]./255;
purple_color = '#472d56';
filler = '#f6f5ec';

hold on;
for i = 1:3
    load(Threename{i});
    StartSec = 3;
    Find_l_StartSec = find(record_time_stamp > StartSec);
    Front = Find_l_StartSec(1);

    EndSec   = 7;
    Find_l_EndSec = find(record_time_stamp > EndSec);
    Rear = Find_l_EndSec(1);

    Cut = Front : Rear;

    c = 1:length(Cut); 


    x = record_p(Cut,1)-record_p(Front,1);
    y = record_p(Cut,2)-record_p(Front,2);
    z = record_p(Cut,3)-record_p(Front,3);

    StartColor = [34 139 34]./256;
    EndColor   = 	[30 144 255]./256;

    colorMap = [linspace(StartColor(1),EndColor(1),256)',...
                linspace(StartColor(2),EndColor(2),256)',...
                linspace(StartColor(3),EndColor(3),256)'];
    filler = '#f6f5ec';

    sample = 1:10:length(z);

    h = patch([x(sample); nan],[y(sample); nan],[z(sample); nan],[c(sample)'; nan], 'edgecolor', 'interp','Linewidth',3, 'EdgeAlpha', 0.6);
%     alpha(.5);


l = light;
l.Color = [1 1 1];
l.Position = [1 0 1];

    scatter3(x(1), y(1),z(1),50, 'Linewidth',1.5, ...
             'MarkerEdgeColor', StartColor, 'MarkerFaceColor',filler);
    scatter3(x(sample(end)), y(sample(end)),z(sample(end)),50, 'Linewidth',1.5, ...
             'MarkerEdgeColor', EndColor, 'MarkerFaceColor',filler);
end
% h = scatter3(x,y,z,20,power);
grid on
colormap(colorMap)  
view([-0.5,-1,1])
light('Position', [1 1 1]);
axis equal;


figure;
curve_width = 1.2;

x_error_all = [];
y_error_all = [];
z_error_all = [];

for i = 1:3
    load(Threename{i});
    StartSec = 3;
    Find_l_StartSec = find(record_time_stamp > StartSec);
    Front = Find_l_StartSec(1);

    EndSec   = 7;
    Find_l_EndSec = find(record_time_stamp > EndSec);
    Rear = Find_l_EndSec(1);

    Cut = Front : Rear;

    alpha=0.5;

    x = record_p(Cut,1)-record_p(Front,1);
    y = record_p(Cut,2)-record_p(Front,2);
    z = record_p(Cut,3)-record_p(Front,3);
    
    x_error_all = [x_error_all; x - 0.5 * (record_time_stamp(Cut)-StartSec)'];
    y_error_all = [y_error_all; y];
    z_error_all = [z_error_all; z];
    subplot(3,1,1); 
    hold on;
    % p1=plot(time_secs, x_pos,'--', 'Linewidth', curve_width_ref, 'color', red_color);
    px(i)=plot(record_time_stamp(Cut)-StartSec,x , ...
        'Linewidth', curve_width, 'color', [red_color,alpha]);
    if i==1
    p2 =  plot(record_time_stamp(Cut)-StartSec,0.5 * (record_time_stamp(Cut)-StartSec),'--' ,...
        'Linewidth', curve_width, 'color', red_color);
%      legend([p2,px(1)],'Reference X pos.','Real X pos.');
    end
    xlim([0, EndSec - StartSec]);
    
    % legend([p1,p3],'Reference X pos.','Real X pos.');

    subplot(3,1,2);
    hold on;
    % p1 = plot(time_secs, y_pos,'--', 'Linewidth', curve_width_ref, 'color', green_color);
    py(i) =plot(record_time_stamp(Cut)-StartSec,y,...
        'Linewidth', curve_width, 'color', [green_color,alpha]);
    if i==1
         p2 = plot(record_time_stamp(Cut)-StartSec,0 * (record_time_stamp(Cut)-StartSec),'--' ,...
        'Linewidth', curve_width, 'color', green_color);
%          legend([p2,py(1)],'Reference Y pos.','Real Y pos.');
    end
    xlim([0, EndSec - StartSec]);
    ylim([-0.5,0.5]);
    % legend([p1,p3],'Reference Y pos.','Real Y pos.');

    subplot(3,1,3);
    hold on;
    % p1=plot(time_secs, z_pos,'--', 'Linewidth', curve_width_ref, 'color', blue_color);
    pz(i)=plot(record_time_stamp(Cut)-StartSec,z,...
        'Linewidth', curve_width, 'color', [blue_color,alpha]);
    if i==1
    p2= plot(record_time_stamp(Cut)-StartSec,0 * (record_time_stamp(Cut)-StartSec),'--' ,...
        'Linewidth', curve_width, 'color', blue_color);
%     legend([p2,pz(1)],'Reference Z pos.','Real Z pos.');
    end
    xlim([0, EndSec - StartSec]);
    ylim([-0.5,0.5]);
end
% ylim([-0.3,.3]);
% legend([p1,p3],'Reference Z pos.','Real Z pos.');

set(gcf, 'Position', [100 100 600 450]); 

disp('Along track error MAX'+string(max(abs(x_error_all))) );
disp('Along track error RMS'+string(rms(abs(x_error_all))) );

disp('Cross track error MAX'+string(max(abs(y_error_all))) );
disp('Cross track error RMS'+string(rms(abs(y_error_all))) );

disp('Altitude error MAX'+string(max(abs(z_error_all))) );
disp('Altitude error RMS'+string(rms(abs(z_error_all))) );  