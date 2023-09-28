figure;
hold on;
for i = 1:3 
        plot(record_time_stamp, record_Gammap(:,i),'linewidth',1);
end
legend('X','Y','Z');
hold off;