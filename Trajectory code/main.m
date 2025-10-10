clc
clear
close all

% Create a new rocket object
myRocket = Rocket();
myRocket.rocketName = 'Rocket';

noseCone = NoseCone('MyNoseCone', 'ogive', 0.5, 0.1, 1.0, []);
myRocket = myRocket.add_part(noseCone);

bodyTube1 = BodyTube('MainBodyTube', 1.5, 0.1, 1.2, 0.003);
myRocket = myRocket.add_part(bodyTube1);

parachute = Parachute('MainParachute', 1.5, 0.8, 300, 0.1, 0.5, bodyTube1);
parachute = parachute.update_position([0, 0, 0.1]); % Place it 10cm from the top of the tube
myRocket = myRocket.add_part(parachute);

avionics = Mass('AvionicsBay', 0.4, 0.15, 0.09, bodyTube1);
avionics = avionics.update_position([0, 0, 0.3]); % Place it 30cm from the top of the tube
myRocket = myRocket.add_part(avionics);

motorTube = BodyTube('MotorTube', 0.5, 0.05, 0.7, 0.002);
myRocket = myRocket.add_part(motorTube);


thrust_curve = [0, 1500; 0.2, 1600; 3.5, 1400; 4.0, 50; 4.1, 0]; % Made up
motor = Motor('MyMotor', 5.0, 2.8, 0.03, 0.6, thrust_curve, motorTube);
motor = motor.update_position([0, 0, 0.05]); % Place it 5cm from the top of the motor tube
myRocket = myRocket.add_part(motor);

finSet = FinSet('MyFins', 4, 0.15, 0.2, 0.1, 0.05, 0.004, 0.1, motorTube);
finSet = finSet.update_position([0, 0, 0.4]); % Attach fins starting 40cm down the motor tube
myRocket = myRocket.add_part(finSet);

% Debug calculations
time = 0;
total_mass = myRocket.get_total_mass(time);
cg = myRocket.get_cg(time);
inertia = myRocket.get_inertia(time);

fprintf('Rocket: %s\n', myRocket.rocketName);
fprintf('Total Mass at t=%.1f s: %.2f kg\n', time, total_mass);
fprintf('Center of Gravity (from nose tip) at t=%.1f s: %.2f m\n', time, cg);
fprintf('Inertia Tensor at t=%.1f s (kg*m^2):\n', time);
disp(inertia);