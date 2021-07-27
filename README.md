# osr_course_attempt
Files for osr_course_attempt:

1.1 fk2dof
1.2 fkdenso
1.3 ik2dof

2.1 triangle_path_puzzle
2.2 tp_PRM
2.3 tp_RRT
2.4 tp_timeparam
2.5 tp_postprocess

3.1 p_and_p

4.1 fdtakeoverall
4.2 solve_transform

Instructions: Run files in order for each section.

Note:
3.1:
-input '0' for 'number of jacobian axis movements (0-5)' for normal and '3' for zero tilt
-max 27 boxes succeeded
-consider decreasing step size for zero tilt to speed up process (more inaccuracies)

4.1:
-finds dimension of glass plane in 2d space
-requires many different parameters to be changed for each image
-consider other methods (logic/function) to find sides of plane
4.2:
-uses the 2d coords of L and R take from 4.1 to find 3d pose and transform for glass plane
