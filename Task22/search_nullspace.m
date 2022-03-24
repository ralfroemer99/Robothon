function success = search_nullspace(robot, config, sphere, q_ns)
success = 0;

% Check all the sampled nullspace configurations for collision. If a collision-free configuration is found, terminate.
for ii = 1:size(q_ns,1)
    config_tmp = config;
    config_tmp(1:7) = q_ns(ii,1:7);
    [isColliding,~,~] = checkCollision(robot,config_tmp,{sphere});
    % If there is no collision with the sphere, the exploration was successful
    if isColliding(2) == 0
        success = 1;
        return;
    end
end

end