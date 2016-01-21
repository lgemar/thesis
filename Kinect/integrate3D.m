function out = integrate3D(t,x)

intx = cumtrapz(t,x(1,:));
inty = cumtrapz(t,x(2,:));
intz = cumtrapz(t,x(3,:));
out = [intx;inty;intz];
