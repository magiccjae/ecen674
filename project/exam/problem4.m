clear;

u = 45;
v = -2;
w = 6;

phi = 5*pi/180;
theta = 10*pi/180;
psi = -27*pi/180;

sp = sin(phi);
cp = cos(phi);
st = sin(theta);
ct = cos(theta);
ss = sin(psi);
cs = cos(psi);

wn = 5;
we = -1;
wd = 0;

R_v_b = [ct*cs   ct*ss   -st;...
         sp*st*cs-cp*ss     sp*st*ss+cp*cs      sp*ct;...
         cp*st*cs+sp*ss     cp*st*ss-sp*cs      cp*ct;...
         ];

wind_body = R_v_b * [wn; we; wd];
u_w = wind_body(1);
v_w = wind_body(2);
w_w = wind_body(3);

airspeed_body = [u-u_w; v-v_w; w-w_w];

u_r = airspeed_body(1);
v_r = airspeed_body(2);
w_r = airspeed_body(3);

Va = sqrt(u_r^2 + v_r^2 + w_r^2)
alpha = atan(w_r/u_r) * 180 / pi
beta = asin(v_r/Va) * 180 / pi
gamma_a = theta*180/pi-alpha