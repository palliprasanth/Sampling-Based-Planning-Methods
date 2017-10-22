function[armplan] = armplanner(envmap, armstart, armgoal, planner_id);


 %call the planner in C
 tic;
 [armplan, armplanlength] = planner(envmap, armstart, armgoal, planner_id);
 toc;

