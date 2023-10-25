clear all;
file_name = "DESKTOP-4J3FS1P_2023_10_09_09_09_29UFlapperInMocapwayvideo";
load(file_name + ".mat")
figure; 
line_color = '#6f599c';

StartTime = 3;
EmdTime   = 15;

cut = find(record_time_stamp >= StartTime & record_time_stamp <= EmdTime);

X_pos = record_p(cut,1) - record_p(cut(1),1);
Y_pos = record_p(cut,2) - record_p(cut(1),2);
Z_pos = record_p(cut,3) - record_p(cut(1),3);

plot3(X_pos, Y_pos, Z_pos,'--',...
      'color',line_color,'Linewidth',1.5);
hold on;
plot3(X_pos(1), Y_pos(1), Z_pos(1),'o');
% zticks([1.5, 1.6]);
R = reshape(record_Flapper_att(cut(1),:,:),3,3);
drawframe([X_pos(1),Y_pos(1),Z_pos(1)], R, 0.2 );
xlim([min(X_pos)-0.2,max(X_pos)+0.2]);
ylim([min(Y_pos)-0.2,max(Y_pos)+0.2]);
zlim([min(Z_pos)-0.2,max(Z_pos)+0.2]);
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
grid on;
hold off;

v = VideoWriter(file_name+".avi");
v.FrameRate=100;
open(v);
axis equal;
axis tight manual;

for k = 1:size(X_pos,1)
    plot3(X_pos, Y_pos, Z_pos,'--',...
          'color',line_color,'Linewidth',1.5);
    hold on;
    plot3(X_pos(k), Y_pos(k), Z_pos(k),'o');
    % zticks([1.5, 1.6]);
    R = reshape(record_Flapper_att(cut(k),:,:),3,3);
    drawframe([X_pos(k),Y_pos(k),Z_pos(k)], R, 0.2 );
    grid on;
    axis equal;
    axis tight manual;
    xlim([min(X_pos)-0.2,max(X_pos)+0.2]);
    ylim([min(Y_pos)-0.2,max(Y_pos)+0.2]);
    zlim([min(Z_pos)-0.2,max(Z_pos)+0.2]);
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    hold off;
    frame = getframe(gcf);
    writeVideo(v,frame)
    pause(0.01);
end

close(v);

