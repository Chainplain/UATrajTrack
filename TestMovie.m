Z = peaks;
surf(Z);
axis tight manual;
set(gca,"NextPlot","replacechildren");
v = VideoWriter("peaks.avi");

v.FrameRate=1;
open(v);
for k = 1:20
   surf(sin(2*pi*k/20)*Z,Z)
   frame = getframe(gcf);
   writeVideo(v,frame)
end
close(v);