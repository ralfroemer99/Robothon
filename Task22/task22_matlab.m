clear; close all
%% Initialization
% Spatial resolution (Task: 0.005 m)
res = 0.005;

% Workspace boundaries
x_lim = [-0.2 0.55];
y_lim = [-0.15 0.15];
z_lim = [0.0 0.9];

x_span = x_lim(1):res:x_lim(2);
y_span = y_lim(1):res:y_lim(2);
z_span = z_lim(1):res:z_lim(2);

% Matrix containing the collision flags for all sphere positions
coll_mat = zeros(length(x_span), length(y_span), length(z_span));

% Add box
box = collisionBox(0.6,0.6,0.6);
box.Pose = trvec2tform([0.7,0,0.3]);

% Add cylinder
cyl = collisionCylinder(0.01,0.06);
cyl.Pose = trvec2tform([0.5,0,0.63]);

% Load panda model and set configuration obtained from task 2.1 in CoppeliaSim
robot = loadrobot('frankaEmikaPanda','DataFormat','column');
config = robot.homeConfiguration;
config(1) = 0.0;
config(2) = -0.561;
config(3) = 0.003;
config(4) = -2.148;
config(5) = 0.0;
config(6) = 2.357;
config(7) = 0.788;
config(8) = 0.01;
config(9) = 0.01;

% Plot the robot and the box
figure(1); 
show(robot, config); hold on
show(box); hold on
show(cyl); hold on
xlim([-0.3 0.6]); ylim([-0.4 0.4]); zlim([0 0.9]); 
title('Robot with different nullspace configurations', 'Fontsize', 18)
xlabel('x'); ylabel('y'); zlabel('z');

% Compute and plot nullspace configurations
[t, q_ns] = nullspace_configs(robot, config, 1);
[t_cf, q_cf_ns] = coll_free_nullspace_configs(robot, config, q_ns, {box});

figure(2)
for ii = 1:7
    plot(t, q_ns(:, ii)); hold on
end
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'); 
% xlim([1 length(t)]); 
title('Joint trajectories of nullspace exploration')

% Compute and plot end-effector pose for the collision-free nullspace configurations
tmp = size(q_cf_ns, 1);
p = zeros(tmp, 3);
for ii = 1:tmp
    % Get robot configuration
    tmp_config = config;
    for k = 1:7
        tmp_config(k) = q_cf_ns(ii, k);
    end
    T = getTransform(robot, tmp_config, 'panda_link1', 'panda_hand');
    p(ii, :) = T(1:3,4)';
    if mod(ii,4) == 0
        figure(1)
        show(robot, tmp_config); hold on
    end
end

figure(3)
for ii = 1:3
    plot(t_cf, p(:, ii)); hold on
end
legend('x', 'y', 'z'); 
xlim([1 length(t_cf)]); 
title('End-effector position for the collision-free nullspace configurations', 'Fontsize', 18)

% Create sphere with radius of 0.02 m
sphere = collisionSphere(0.02);

%% Check collision for every possible sphere position
tic
for ii = 1:length(x_span)
    for jj = 1:length(y_span)
        for kk = 1:length(z_span)
            x_pos = x_span(ii); y_pos = y_span(jj); z_pos = z_span(kk);
            T = trvec2tform([x_pos y_pos z_pos]);
            sphere.Pose = T;
            [isColliding,~,~] = checkCollision(robot,config,{sphere, box});
            if isColliding(2) == 1
                if search_nullspace(robot, config, sphere, q_cf_ns) == 1
                    collision_flag = 2;
                else
                    collision_flag = 1;  
                end
            else
                collision_flag = 0;
            end
            coll_mat(ii,jj,kk) = collision_flag;
            fprintf('x = %.3f | y = %.3f | z = %.3f | collision = %.0f \n', [x_pos, y_pos, z_pos, double(collision_flag)]);
        end
    end
end
elapsed_time = toc;
sprintf('Finished collision checking after t = %.2f seconds', elapsed_time)

eval_coll_mat(coll_mat);

%% Plot collision points in 3D
figure(4)
show(robot,config); hold on
for ii = 1:length(x_span)
    for jj = 1:length(y_span)
        for kk = 1:length(z_span)
            x_pos = x_span(ii); y_pos = y_span(jj); z_pos = z_span(kk);
            if coll_mat(ii,jj,kk) == 0
%                 plot3(x_pos, y_pos, z_pos, 'Marker', 'o', 'Color', 'green'); hold on
            elseif coll_mat(ii,jj,kk) == 1
                plot3(x_pos, y_pos, z_pos, 'Marker', 'o', 'Color', 'red', 'LineWidth',0.5); hold on
            else
                plot3(x_pos, y_pos, z_pos, 'Marker', 'o', 'Color', 'blue', 'LineWidth',0.5); hold on
            end
        end
    end
end
grid on; 
xlabel('x'), ylabel('y'); zlabel('z'); 
title('Initial configuration with the checked sphere positions', 'Fontsize', 18)
xlim([-0.3 0.6]); ylim([-0.3 0.3]); zlim([0 0.9]);
savefig('figs/all_sphere_pos_05cm.fig');
