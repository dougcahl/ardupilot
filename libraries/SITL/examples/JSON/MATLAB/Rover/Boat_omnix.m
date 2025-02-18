clc
clear
close all

% ArduCopter SITL example vehicle
% This sets up the vehicle properties that are used in the simulation
% simple 450 size x quad based on Hexsoon

% setup the motors

% locations (xyz) in m based on omni x
motor_radius = 0.5; % 1 meter
motor(1).location = [[cosd(45),  sind(45)]*motor_radius,0]; % front right
motor(2).location = [[cosd(-45), sind(-45)]*motor_radius,0]; % rear right
motor(3).location = [[cosd(-135),sind(-135)]*motor_radius,0]; % rear left
motor(4).location = [[cosd(135), sind(135)]*motor_radius,0]; % front left
% motor(1).location = [[sind(45),cosd(45)]*motor_radius,0]; % front right
% motor(2).location = [[sind(135),cosd(135)]*motor_radius,0]; % rear right
% motor(3).location = [[sind(225),cosd(225)]*motor_radius,0]; % rear left
% motor(4).location = [[sind(315),cosd(315)]*motor_radius,0]; % front left
rover.motor_radius = motor_radius;

% motor thrust direction
motor(1).thrustdir = 135; % towards front left
motor(2).thrustdir = 45;  % towards front right
motor(3).thrustdir = 135; % towards front left
motor(4).thrustdir = 45;  % towards front right

% PWM output to use
% set to match OMNIX 
% https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html
motor(1).channel = 1;
motor(2).channel = 2;
motor(3).channel = 3;
motor(4).channel = 4;

% rotation direction: 1 = cw, -1 = ccw
% motor(1).direction = -1;
% motor(2).direction = 1;
% motor(3).direction = -1;
% motor(4).direction = 1;

% motor properties
electrical.kv = 880; % (rpm/volt)
electrical.no_load_current = [0.7,10]; % (A) @ (V)
electrical.resistance = 0.115; % (ohms)

% ESC properties
esc.resistance = 0.01; % (ohms)

% Propeller properties
prop.diameter = 245 * 0.001; % (m)
prop.pitch = 114.3 * 0.001; % (m)
prop.num_blades = 2;
prop.PConst = 1.13;
prop.TConst = 1;
prop.mass = 12.5*0.001; % (kg) (only used for inertia)
prop.inertia = (1/12)*prop.mass*prop.diameter^2; % rotational inertia (kgm^2) (rod about center)

% assign properties to motors
for i = 1:4
    motor(i).electrical = electrical;
    motor(i).esc = esc;
    motor(i).prop = prop;
end

% Setup battery
battery.voltage = 4*4.2; % (volts)
battery.resistance = 0.0034; % (ohms)
battery.capacity = 5.2; % (ah)

% Add all to vehicle
rover.motors = motor;
rover.battery = battery;
% rover.mass = 2; % (kg)
rover.mass = 50; % (kg)
inertia = (2/5) * rover.mass * (1)^2; % (sphere)
rover.inertia = diag(ones(3,1)*inertia); % rotational inertia matrix (kgm^2) 
rover.cd = [0.5;0.5;0.5];
rover.cd_ref_area = [1;1;1] * pi * (1)^2;

save('robocat','rover')

% Plot motor curves
% http://www.bavaria-direct.co.za/constants/
% http://www.stefanv.com/rcstuff/qf200204.html
% Some calculators estimate heat and increase resistance with temp
% But then we have to estimate the power dissipation
% Max power for plot only
max_power = 260;
battery.voltage = battery.voltage * 0.50;

Kt = 1/(electrical.kv * ((2*pi)/60) ); % Convert Kv to rads/second

% plot the current from 0 to max power
amps = 0:0.1:max_power/battery.voltage;
power_in = amps * battery.voltage;

% voltage drop due to copper and esc
copper_drop = amps * electrical.resistance; 
esc_drop = amps * esc.resistance;

ideal_voltage = battery.voltage - copper_drop - esc_drop;
power_out = ideal_voltage .* (amps - electrical.no_load_current(1));
efficiency = power_out ./ power_in;

torque = Kt * amps;
rpm = ideal_voltage * electrical.kv;

% Plot motor characteristics
figure('name',sprintf('motor characteristics at %0.2f volts',battery.voltage))
subplot(2,2,1)
hold all
title('RPM')
plot(amps,rpm)
xlabel('Current (A)')
ylabel('RPM')
xlim([0,amps(end)])


subplot(2,2,2)
hold all
title('torque')
plot(amps,torque)
xlabel('Current (A)')
ylabel('torque (NM)')
xlim([0,amps(end)])

subplot(2,2,3)
hold all
title('power')
plot(amps,power_in)
plot(amps,power_out)
xlabel('Current (A)')
ylabel('power (W)')
ylim([0,inf])
xlim([0,amps(end)])
legend('Power in','Power out','location','northwest')

subplot(2,2,4)
hold all
title('efficiency')
plot(amps,efficiency)
xlabel('Current (A)')
ylabel('efficiency (%)')
ylim([0,inf])
xlim([0,amps(end)])
