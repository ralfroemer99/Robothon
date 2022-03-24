function [t_cf, q_cf_ns] = coll_free_nullspace_configs(robot, config, q_ns, coll_objects)
q_cf_ns = [];
for ii = 1:size(q_ns,1)
    config_tmp = config;
    config_tmp(1:7) = q_ns(ii,1:7);
    [isColliding,~,~] = checkCollision(robot,config_tmp,coll_objects);
    if ~isColliding(2)
        q_cf_ns = [q_cf_ns; config_tmp(1:7)'];
    end
end
t_cf = 1:size(q_cf_ns,1);
