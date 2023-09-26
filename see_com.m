figure;
hold on;
legends = {'throtle','roll','pitch','yaw'};
for i = 1:4 
        subplot(4,1,i)
        plot(record_time_stamp,record_com(:,i))
        legend(legends{i})
end
% legend('throtle','roll','pitch','yaw');