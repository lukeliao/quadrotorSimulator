% Test and plot trajectories

t_start = 0;
t_end = 15;
t_inc = 0.01;
t_v = 0:t_inc:t_end;

pos_v = zeros(length(t_v),3);
vel_v = zeros(length(t_v),3);
acc_v = zeros(length(t_v),3);

for i = 1:length(t_v)
    states = trajectory_generator(t_v(i), 1);
    pos_v(i,:) = states.pos';
    vel_v(i,:) = states.vel';
    acc_v(i,:) = states.acc';
end

figure(4);
subplot(2,2,1);
plot(t_v, pos_v);
title('Position');
legend('x','y','z');
grid on;

subplot(2,2,2);
plot(t_v, vel_v);
title('Velocity');
legend('x','y','z');
grid on;

subplot(2,2,3);
plot(t_v, acc_v);
title('Acceleration');
legend('x','y','z');
grid on;

subplot(2,2,4);
plot3(pos_v(:,1), pos_v(:,2), pos_v(:,3), 'r-')
title('Trajectory');
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

set(gcf, 'color', 'white');
