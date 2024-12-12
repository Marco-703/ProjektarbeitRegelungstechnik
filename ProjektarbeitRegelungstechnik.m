close all;
clear;
clc;


% Input: Objekt, Masse, Dimensionen
% Parameter: initiale Höhe, Sampling Rate


g = 9.81;

duration = 20.0;
samplingRate = 100.0;

initialHeight = 500;
mass = 2;
radius = 0.2;


state = [initialHeight; 0; 0];
stepDuration = 1 / samplingRate;
steps = duration / stepDuration;

stateLogger = repmat([0; 0; 0], 1, steps);
dragForce = [];

for i = 1:steps
    velocity = state(2,1);
    force = ball_acceleration(radius, velocity);
    acceleration = (-1.0 * g) + (force / mass);

    state = discreteStateSpace(state, acceleration, stepDuration);

    stateLogger(:, i) = state; % Store the state at each step
    dragForce(i) = force;

end


% Plot the height over time
time = (0:steps-1) * stepDuration; % Create a time vector
% Plot height over time
figure;
plot(time, stateLogger(1, :)); % Height is the first row
xlabel('Time (s)');
ylabel('Height (m)');
title('Height vs Time');
grid on;

% Plot velocity over time
figure;
plot(time, stateLogger(2, :)); % Velocity is the second row
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs Time');
grid on;

% Plot acceleration over time
figure;
plot(time, stateLogger(3, :)); % Acceleration is the third row
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration vs Time');
grid on;

figure;
plot(time, dragForce);


%{
% Main script to get user input and display the drag force
mass = input('Enter the mass of the ball (kg): ');
radius = input('Enter the radius of the ball (m): ');
v = input('Enter the velocity of the ball (m/s): ');


% Calculate drag force using the function
F_drag = ball_acceleration(mass, radius, v);
% Display the result
fprintf('The drag force acting on the ball is %.2f N\n', F_drag);
%}





function newState = discreteStateSpace(state, u, dt)
    A = [1,dt,0.5*dt*dt; 0,1,dt; 0,0,1];
    B = [0; dt; 0];

    newState = A * state + B * u;


end


function F_drag = ball_acceleration(radius, v)
    % Constants
    C_d = 0.47; % Drag coefficient for a sphere
    g = 9.81;
    rho = 1.225; % Air density (kg/m^3) at sea level

    % Calculate the reference area (A = pi * r^2)
    A = pi * radius^2;


    % Drag force calculation
    F_drag = 0.5 * rho * v^2 * C_d * A;
end