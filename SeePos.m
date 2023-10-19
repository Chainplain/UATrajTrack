figure;

for i = 1:3 
        subplot(3,1,i)
        plot(record_time_stamp,record_p(:,i))
        hold on;
        plot(record_time_stamp,record_p(:,i))
end

figure;
for i =1:3
    subplot(3,1,i)
        plot(record_time_stamp,record_v(:,i));
        title('velocity');
end

cut = 330:800;
figure; 
plot3(record_p(cut,1),record_p(cut,2),record_p(cut,3));
hold on;
plot3(record_p(cut(1),1),record_p(cut(1),2),record_p(cut(1),3),'o');
axis equal;

