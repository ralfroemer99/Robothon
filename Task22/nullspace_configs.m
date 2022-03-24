function [t, q_ns] = nullspace_configs(robot, config, scene)
% Define sufficiently long timespan
tspan = 0:1:22;

% Initialize the state with the robot configuration
q0 = config(1:7);
[t, q] = ode23(@(t, q) sysfunc(t, q, robot, scene), tspan, q0);
q_ns = mod(q+pi, 2*pi)-pi;
end

function dq = sysfunc(~, q, robot, scene)
% Get current joint values
config = zeros(9,1);
for k = 1:7
    config(k) = q(k);
end
% Finger joints are not included on the ODE simulation as they are constant within the whole nullspace
config(8) = 0.01;
config(9) = 0.01;

% Compute end-effector Jacobian
J = geometricJacobian(robot, config, 'panda_hand');
J = J(:,1:7);

% Compute matrix projecting every vector into the nullspace of J
A = (eye(7) - J'/(J*J')*J);

% Compute suitable vector b
% Find row with non-zero values to generate a sufficiently large joint velocity
row = 7;
while norm(A(row, :)) <= 1e-2
    row = row-1;
end
if row == 7
    b = lsqlin(A(7,:), 1);
else
    % Ensure that the returned vector does not yield dq_1 < 0 (then the robot would move backwards)
    options = optimset('Display', 'off');
    b = quadprog(0.5*A(row,:)'*A(row,:), -2*A(row,:)', -A(7,:), -1e-5, [], [], [], [], [], options);
end
if scene == 1
    b = [1; 0; 0; 0; 0; 0; 0]; % Turned out to be suitable for exploring the whole nullspace
elseif scene == 2
    b = [0; 0; 1; 0; 0; 0; 0];
end
% Compute joint velocities
dq = A*b;
end
