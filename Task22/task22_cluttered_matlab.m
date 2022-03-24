clear; close all
%% Initialization
% Spatial resolution (Task: 0.005 m)
res = 0.005;

% Workspace boundaries
x_lim = [-0.3 0.2];
y_lim = [-0.1 0.9];
z_lim = [0.0 0.4];

x_span = x_lim(1):res:x_lim(2);
y_span = y_lim(1):res:y_lim(2);
z_span = z_lim(1):res:z_lim(2);

% Matrix containing the collision flags for all sphere positions
coll_mat = zeros(length(x_span), length(y_span), length(z_span));

% Add boxes
box{1} = collisionBox(0.2,0.2,0.5);
box{1}.Pose = trvec2tform([0.2,0.5,0.25]);
box{2} = collisionBox(0.25,0.25,1.75);
box{2}.Pose = trvec2tform([-0.55,-0.55,0]);
box{3} = collisionBox(0.28,0.28,0.22);
box{3}.Pose = trvec2tform([0.5,-0.55,0.14]);
box{4} = collisionBox(0.25,0.25,1.1);
box{4}.Pose = trvec2tform([-0.4,0.4,0.5]);

% Add cylinder
cyl = collisionCylinder(0.01,0.06);
cyl.Pose = trvec2tform([0.15,0.725,0.2]);

% Load panda model and set configuration obtained from task 2.1 in CoppeliaSim
robot = loadrobot('frankaEmikaPanda','DataFormat','column');
config = robot.homeConfiguration;
config(1) = 2.409;
config(2) = 1.732;
config(3) = -1.468;
config(4) = -1.336;
config(5) = 0.055;
config(6) = 2.080;
config(7) = 2.508;
config(8) = 0.01;
config(9) = 0.01;

% Plot the robot and the boxes
figure(1); 
show(robot, config); hold on
for ii = 1:4
    show(box{ii}); hold on
end
show(cyl); hold on
xlim([-0.8 0.8]); ylim([-0.8 0.9]); zlim([0 0.9]); 
title('Robot with different nullspace configurations', 'Fontsize', 18)
xlabel('x'); ylabel('y'); zlabel('z');

% Compute and plot nullspace configurations
[t, q_ns] = nullspace_configs(robot, config, 2);
[t_cf, q_cf_ns] = coll_free_nullspace_configs(robot, config, q_ns, box);

figure(2)
for ii = 1:7
    plot(t, q_ns(:, ii)); hold on
end
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7'); 
% xlim([0 length(t)]); 
title('Joint trajectories of nullspace exploration', 'Fontsize', 18)

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
    if mod(ii,8) == 0
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
            [isColliding,~,~] = checkCollision(robot,config,{sphere, box{1},box{2},box{3},box{4}});
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
xlim([-0.4 0.3]); ylim([-0.2 0.9]); zlim([0 0.5]);
savefig('figs/all_sphere_pos_05cm_cluttered.fig');