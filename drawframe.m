function [] = drawframe(Origin,Attitude_mat,Length)
e_1 = [1,0,0]';
e_2 = [0,1,0]';
e_3 = [0,0,1]';

X = Length * Attitude_mat * e_1;
Y = Length * Attitude_mat * e_2;
Z = Length * Attitude_mat * e_3;

Linewidth = 2;

hold on;
quiver3(Origin(1),Origin(2),Origin(3),X(1),X(2),X(3),'color','#d71345','linewidth',Linewidth);
quiver3(Origin(1),Origin(2),Origin(3),Y(1),Y(2),Y(3),'color','#1d953f','linewidth',Linewidth);
quiver3(Origin(1),Origin(2),Origin(3),Z(1),Z(2),Z(3),'color','#009ad6','linewidth',Linewidth);
scatter3(Origin(1),Origin(2),Origin(3), 'k', 'filled')
end

