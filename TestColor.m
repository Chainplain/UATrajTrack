theta = linspace(0, 2*pi, 100);
z = linspace(0, 5, 100);
r = z .* sin(theta);

% 3D plot with enhanced depth perception
figure;
colormap('jet'); % Set the colormap
h = plot3(r.*cos(theta), r.*sin(theta), z, 'LineWidth', 2);
view(45, 30); % Adjust the viewing angle
light('Position', [1 1 1]); % Add a light source
title('Enhanced 3D Plot');
xlabel('X');
ylabel('Y');
zlabel('Z');