figure;

subplot(2,3,1)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,2), 'r')
title('X coordinate [m]');

subplot(2,3,2)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,3), 'r')
title('Y coordinate [m]');

subplot(2,3,3)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,4), 'r')
title('Heading [rad]');

subplot(2,3,5)
plot(out.STATES_SAMPLED(:,2), out.STATES_SAMPLED(:,3), 'r')
title('Trajectory');
xlabel('X coordinate [m]');
ylabel('Y coordinate [m]');

figure;

subplot(1,2,1)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Linear velocity [m/s]');
xlabel('Time [s]');
ylabel('Velocity [m/s]');

subplot(1,2,2)
plot(out.CONTROLS(:,1), out.CONTROLS(:,3), 'r')
title('Heading turn rate [m/s]');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');