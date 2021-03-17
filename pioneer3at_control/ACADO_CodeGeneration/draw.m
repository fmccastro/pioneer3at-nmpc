if toSave == 1
    string = strcat('Ts = ', num2str(Ts), '___N = ', num2str(N), '___Integrator = ', num2str(Integrator), '___Ns = ', num2str(Ns), '___Reference = ', num2str(ref), '__WithOnlineData.' );

    cd ../ACADO_simulations
        mkdir(string);
    cd(string)
end

figure;

subplot(2,3,1)
plot(timeArray', trajectory(:, 1), 'r')
title('X coordinate [m]');
xlabel('Time [s]');
ylabel('X [m]');

subplot(2,3,2)
plot(timeArray', trajectory(:, 2), 'r')
title('Y coordinate [m]');
xlabel('Time [s]');
ylabel('Y [m]');

subplot(2,3,3)
plot(timeArray', trajectory(:, 3), 'r')
title('Heading [rad]');
xlabel('Time [s]');
ylabel('Heading [rad]');

subplot(2,3,5)
plot(trajectory(:, 1), trajectory(:, 2), 'r')
title('Trajectory');
xlabel('X coordinate [m]');
ylabel('Y coordinate [m]');

if toSave == 1
    gtext(['Time to reach goal point: ', newline, num2str(timeToGoal), ' seconds.'], 'FontSize', 10);
    saveas(gcf, 'States.png');
end

figure;

subplot(1,2,1)
plot(timeArray', commands(:, 1), 'r')
title('Linear velocity [m/s]');
xlabel('Time [s]');
ylabel('Velocity [m/s]');

subplot(1,2,2)
plot(timeArray', commands(:, 2), 'r')
title('Heading turn rate [m/s]');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');

if toSave == 1
    saveas(gcf, 'Controls.png');
    cd ../../ACADO_CodeGeneration
end
