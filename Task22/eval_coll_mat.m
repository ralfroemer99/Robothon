function eval_coll_mat(coll_mat)
    % Check for how many of the sphere positions colliding with the initial configuration, a collision-free configuration 
    % could be found in the nullspace
    [nx,ny,nz] = size(coll_mat);
    fprintf('Checked %.0f sphere positions. \n', nx*ny*nz);
    n_init_coll = sum(coll_mat >= 1, 'all');
    n_avoid_coll = sum(coll_mat == 2, 'all');
    fprintf('For %.0f sphere positions, there is a collision with the initial robot configuration. \n', n_init_coll);
    fprintf('For %.0f of these sphere positions, the collision can be avoided by exploring the nullspace. \n', n_avoid_coll);
    fprintf('Thus, %.1f %% of the collisions can be avoided. \n', n_avoid_coll/n_init_coll*100);
end

