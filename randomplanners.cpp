#include <math.h>
#include "mex.h"
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <iterator>
#include <ctime>
#include <chrono>
#include "graphheader.hpp"
#include "plannerheader.hpp"

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

#define EPS 0.25

/*
Trapped = -1
Advanced = 0
Reached = 1
*/

vector<int> extend_rrt(Graph* rrt_tree, double* map, int x_size, int y_size, double* current_sample_angles, int numofDOFs){
	vector<int> extend_return = {-1,1,1};
	int cur_node_id, nearest_node_id;
	double nearest_dist;
	double current_eps_step_angles[5], nearest_node_angles[5], difference_angles[5];
	int arm_movement_sign[5]; // To keep track of nearest motion to be clockwise or anti-clockwise

	nearest_node_id = rrt_tree->getNearestNeighbour(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
	convert_to_unit(difference_angles, numofDOFs);

	if (nearest_dist < EPS){
				// collision check added here
		if(IsValidArmConfiguration(current_sample_angles, numofDOFs, map, x_size, y_size)){
			cur_node_id = rrt_tree->add_Vertex_ret_id(current_sample_angles[0], current_sample_angles[1], current_sample_angles[2], current_sample_angles[3], current_sample_angles[4], nearest_node_id);			
			rrt_tree->add_Edge(nearest_node_id, cur_node_id);
			extend_return[0] = 1;
			extend_return[1] = nearest_node_id;
			extend_return[2] = cur_node_id;
			return extend_return;
		}
		else{
			extend_return[0] = -1;
			return extend_return;
		}
	}

	else{
		for (int k =0; k<numofDOFs; ++k){
			current_eps_step_angles[k] = nearest_node_angles[k] + EPS*arm_movement_sign[k]*difference_angles[k];
		}
		if(IsValidArmConfiguration(current_eps_step_angles, numofDOFs, map, x_size, y_size)){
			cur_node_id = rrt_tree->add_Vertex_ret_id(current_eps_step_angles[0], current_eps_step_angles[1], current_eps_step_angles[2], current_eps_step_angles[3], current_eps_step_angles[4], nearest_node_id);
			rrt_tree->add_Edge(nearest_node_id, cur_node_id);
			extend_return[0] = 0;
			return extend_return;
		}
		else{
			extend_return[0] = -1;
			return extend_return;
		}
	}
}

void build_rrt(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength, int* planner_id){
	
	const int max_iter = 50000;
	double bias_random;
	int cur_node_id, nearest_node_id;
	double nearest_dist;
	double current_sample_angles[5],current_eps_step_angles[5], nearest_node_angles[5], difference_angles[5];
	int arm_movement_sign[5]; // To keep track of nearest motion to be clockwise or anti-clockwise
	vector<int> extend_track;
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Graph rrt;
	rrt.add_Vertex(armstart_anglesV_rad[0], armstart_anglesV_rad[1], armstart_anglesV_rad[2], armstart_anglesV_rad[3], armstart_anglesV_rad[4]);
	
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> bias_distribution(0.0,1.2);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);

	for (int i=0; i<max_iter; ++i){
		bias_random = bias_distribution(generator);
		if (bias_random > 1){
			extend_track = extend_rrt(&rrt, map, x_size, y_size, armgoal_anglesV_rad, numofDOFs);
			if (extend_track[0] == 1){
				mexPrintf("Found a path with expansion %d \n",cur_node_id);

				int parentid = extend_track[1];
				list<Node*> RRT_Path;
				Node* temp_Node;
				while (parentid != 0){
					temp_Node = rrt.get_Vertex(parentid);
					RRT_Path.push_front(temp_Node);
					parentid = temp_Node->parent_id;
				}
				int numofsamples = RRT_Path.size();
				*plan = (double**) malloc(numofsamples*sizeof(double*));
				int p = 0;
				for (list<Node*>::iterator it = RRT_Path.begin(); it != RRT_Path.end(); it++){
					(*plan)[p] = (double*) malloc(numofDOFs*sizeof(double)); 
					(*plan)[p][0] = (*it)->theta_1;
					(*plan)[p][1] = (*it)->theta_2;
					(*plan)[p][2] = (*it)->theta_3;
					(*plan)[p][3] = (*it)->theta_4;
					(*plan)[p][4] = (*it)->theta_5;
					p++;
				}    
				*planlength = numofsamples;
				return;
			}
		}
		else{
			for (int j =0; j<numofDOFs; ++j){
				current_sample_angles[j] = 2*PI*uni_distribution(generator);
			}
			extend_track = extend_rrt(&rrt, map, x_size, y_size, current_sample_angles, numofDOFs);
		}	
	}
	//rrt.print_Vertices();
	mexPrintf("Did not find a path \n");
	return;
}


void build_rrt_connect(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength, int* planner_id){

	const int max_iter = 50000;
	double bias_random;
	int cur_node_id, nearest_node_id;
	double nearest_dist;
	double current_sample_angles[5],current_eps_step_angles[5], nearest_node_angles[5], difference_angles[5];
	int arm_movement_sign[5]; // To keep track of nearest motion to be clockwise or anti-clockwise
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	bool swap_trees = true;

	Graph rrtA, rrtB;
	rrtA.add_Vertex(armstart_anglesV_rad[0], armstart_anglesV_rad[1], armstart_anglesV_rad[2], armstart_anglesV_rad[3], armstart_anglesV_rad[4]);
	rrtB.add_Vertex(armgoal_anglesV_rad[0], armgoal_anglesV_rad[1], armgoal_anglesV_rad[2], armgoal_anglesV_rad[3], armgoal_anglesV_rad[4]);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);

	for (int i=0; i<max_iter; ++i){

		for (int j =0; j<numofDOFs; ++j){
			current_sample_angles[j] = 2*PI*uni_distribution(generator);
		}

		if (swap_trees){
			nearest_node_id = rrtA.getNearestNeighbour(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
			convert_to_unit(difference_angles, numofDOFs);

			swap_trees = !swap_trees;
		}
		else{

			nearest_node_id = rrtB.getNearestNeighbour(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
			convert_to_unit(difference_angles, numofDOFs);



			swap_trees = !swap_trees;
		}
	}

	return;
}