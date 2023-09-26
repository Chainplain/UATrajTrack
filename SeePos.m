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

