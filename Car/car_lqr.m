Tc = 2;         % Time Constant
m = 1558;       % Vehicle Mass
lf = 1.462884;  % Distance from cg to front axle
lr = 1.405516;  % Distance from cg to rear axle
l = lf + lr;    

Cf = 1.432394487827058e+05; %
Cr = 2.214094963126969e+05;
g = 9.80655;

% %Linearized State Space Matrices
% A = [0, 0, 0, 1, 0;
%      0, 0, 5, 0, 2.2654;
%      0, 0, 0, 0, 1.7171;
%      0, 0, 0, -0.0013, 0;
%      0, 0, 0, 0, -1];

A = [0, 0, 0, 1, 0;
     0, 0, 50, 0, -49.9566;
     0, 0, 0, 0, 6.69245;
     0, 0, 0, -0.0013, 0;
     0, 0, 0, 0, -0.8];
         
B= [0, 0;
    0, 0;
    0, 0;
    2, 0;
    0, 0.8];


Q = diag([0.0000001, 1, 10, 10, 10]); %Cost on States
R = diag([1, 1]);            % Cost on Actions

%LQR
[K, ~, ~] = lqr(A, B, Q, R);
disp(K);

x0 = [0; 0; pi/2; 0; 0];
tspan = 0:0.01:10;


[t, X] = ode45(@(t, x) closedloopdyn(t, x, K), tspan, x0);

%plot


close all;
figure(1);
clf;
plot(t, X(:, 3), 'r', 'LineWidth', 1.5);
hold on;
plot(t, X(:, 5), 'b', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Heading(rad)/ Steering Angle (rad)');

yyaxis right;
plot(t, X(:, 4), 'g', 'LineWidth', 1.5);
plot(t, X(:, 2), 'p', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Lateral Position');
legend('Heading', 'Steering Angle', 'Lateral Position');
title('Car Control');
grid on;
hold off;

% Set up the figure for visualization
figure(2);
axis equal;
grid on;

% Visualization parameters
car_length = 1;
car_width = 2;

% Simulate and visualize the car dynamics
for i = 1:length(t)
    % Extract states from X
    x_pos = X(i, 1);
    y_pos = X(i, 2);
    heading = X(i, 3);
    delta = X(i, 5);
    alpha = atan((car_width/2)/car_length/2);

    % Calculate the car's corner points based on the steering angle
    front_left = [x_pos + cos(heading+alpha)*sqrt(car_width^2+car_length^2), y_pos + sin(heading+alpha)*sqrt(car_width^2+car_length^2)];
    front_right = [x_pos + cos(heading-alpha)*sqrt(car_width^2+car_length^2), y_pos + sin(heading-alpha)*sqrt(car_width^2+car_length^2)];
    rear_left = [x_pos - cos(heading-alpha)*sqrt(car_width^2+car_length^2), y_pos - sin(heading-alpha)*sqrt(car_width^2+car_length^2)];
    rear_right = [x_pos - sin((pi/2-heading)-alpha)*sqrt(car_width^2+car_length^2), y_pos - cos((pi/2-heading)-alpha)*sqrt(car_width^2+car_length^2)];

    % Clear previous visualization
    clf;

    % Plot car body
    car_body = fill([front_left(1), front_right(1), rear_right(1), rear_left(1)], ...
                    [front_left(2), front_right(2), rear_right(2), rear_left(2)], 'b');

    % Update plot properties
    xlim([x_pos - 20, x_pos + 20]);
    ylim([y_pos - 20, y_pos + 20]);
    xlabel('X Position');
    ylabel('Y Position');
    title('Car Simulation');
    grid on;
    drawnow; % Update the plot

    % fixed_xlim = [-20, 20];
    % fixed_ylim = [-20, 20];
    % grid on;
    % 
    % xlim(fixed_xlim);
    % ylim(fixed_ylim);
    % 
    % % Update plot properties
    % xlabel('X Position');
    % ylabel('Y Position');
    % title('Car Simulation');
    % drawnow; % Update the plot


    pause(0.01);
end

function dxdt = closedloopdyn(~, x, K)

    Tc = 2;         % Time Constant
    m = 1558;       % Vehicle Mass
    lf = 1.462884;  % Distance from cg to front axle
    lr = 1.405516;  % Distance from cg to rear axle
    l = lf + lr;    
    
    Cf = 1.432394487827058e+05; %
    Cr = 2.214094963126969e+05;
    g = 9.80655;

    max_del_dot = 10;
    max_throttle = 10;

    v_long = x(4);
    heading = x(3);
    delta = x(5);
    u_bar = [0; 0];

    x_bar = [0; 0; 0; 50; 0];

    delta_x = x-x_bar;
   
    u = u_bar -K*delta_x;


    throttle = u(1);
    delta_cmd= u(2);
    % throttle = 10;
    % steering_torque= 1;


    % if throttle > max_throttle
    %     throttle  = max_throttle;      
    % end
    % if throttle < -max_throttle
    %     throttle = -max_throttle;      
    % end

    f1 = v_long*cos(heading)-((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*sin(heading);
    f2 = v_long*sin(heading)+((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*cos(heading);
    f3 = tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g);
    f4 = -Tc/m*v_long+Tc*throttle;
    f5 = 0.8*(delta_cmd-delta);

    % if f5 > max_del_dot
    %     f5 = max_del_dot;      
    % end
    % if f5 < -max_del_dot
    %     f5 = -max_del_dot;      
    % end


            


    dxdt = [f1; f2; f3; f4; f5];


end
     