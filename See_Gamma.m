figure;
hold on;
for i = 1:3 
        plot(record_time_stamp, record_Gamma(:,i),'linewidth',1);
end
legend('X','Y','Z');