

syms p_long p_lat heading v_long delta throttle delta_cmd;

Tc = 2;         % Time Constant
m = 1558;       % Vehicle Mass
lf = 1.462884;  % Distance from cg to front axle
lr = 1.405516;  % Distance from cg to rear axle
l = lf + lr;    

Cf = 1.432394487827058e+05; %
Cr = 2.214094963126969e+05;
g = 9.80655;



% f1 = v_long*cos(heading)-((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*sin(heading);
% f2 = v_long*sin(heading)+((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*cos(heading);
% f3 = tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g);
% f4 = -Tc/m*v_long+Tc*throttle;
% f5 = 1*(steering_torque-delta);

kus = (m*g*lr/(l*Cf)-m*g*lf/(l*Cr));    
yr = tan(delta)*v_long/(l+kus*v_long^2/g);
v_lat = yr*(lr-m*v_long^2*lr/(Cr*l));

f1 = v_long*cos(heading)-((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*sin(heading);
f2 = v_long*sin(heading)+((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*cos(heading);
f3 = tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g);
f4 = -Tc/m*v_long+Tc*throttle;
f5 = 0.8*(delta_cmd-delta);



% Set equilibrium values
p_long_eq = 0; % Equilibrium value for longitudinal position
p_lat_eq = 0;  % Equilibrium value for lateral position
heading_eq = 0; % Equilibrium value for heading
v_long_eq = 100; % Replace with your desired constant speed
delta_eq = 0; % Equilibrium value for steering angle

throttle_eq = 0; % Equilibrium value for throttle
delta_cmd_eq = 0; % Equilibrium value for steering torque


% Compute the linearized dynamics
A = jacobian([f1, f2, f3, f4, f5], [p_long,p_lat, heading, v_long, delta]);
A = subs(A, [p_long, p_lat, heading, v_long, delta, throttle, steering_torque], ...
            [p_long_eq, p_lat_eq, heading_eq, v_long_eq, delta_eq, throttle_eq, steering_torque_eq]);


B = jacobian([f1, f2, f3, f4, f5], [throttle, delta_cmd]);

B = subs(B, [p_long, p_lat, heading, v_long, delta, throttle, delta_cmd], ...
            [p_long_eq, p_lat_eq, heading_eq, v_long_eq, delta_eq, throttle_eq, delta_cmd_eq]);


% Convert symbolic expressions to numeric matrices
A = double(A);
B = double(B);

disp("A");
disp(A);
disp("B");
disp(B);

