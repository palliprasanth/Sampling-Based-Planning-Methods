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

void build_rrt(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength, int* planner_id){
	
	const int max_iter = 50000;
	const double epsilon = 0.25;
	double bias_random;
	int cur_node_id, nearest_node_id;
	double nearest_dist;
	double current_sample_angles[5],current_eps_step_angles[5], nearest_node_angles[5], difference_angles[5];
	int arm_movement_sign[5]; // To keep track of nearest motion to be clockwise or anti-clockwise
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
			nearest_node_id = rrt.getNearestNeighbour(armgoal_anglesV_rad, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
			convert_to_unit(difference_angles, numofDOFs);
			//for (int m =0; m<numofDOFs; ++m){
			//	mexPrintf("Node Angles: %f", nearest_node_angles[m]);
			//}
			//mexPrintf("\n");

			if (nearest_dist < epsilon){
				// might need to add collision check

				if(IsValidArmConfiguration(current_sample_angles, numofDOFs, map, x_size, y_size)){
					cur_node_id = rrt.add_Vertex_ret_id(armgoal_anglesV_rad[0], armgoal_anglesV_rad[1], armgoal_anglesV_rad[2], armgoal_anglesV_rad[3], armgoal_anglesV_rad[4], nearest_node_id);			
					rrt.add_Edge(nearest_node_id, cur_node_id);
					mexPrintf("Found a path with expansion %d \n",cur_node_id);

					int parentid = nearest_node_id;
					list<Node*> RRT_Path;
					Node* temp_Node;
					while (parentid != 0){
						temp_Node = rrt.get_Vertex(parentid);
						RRT_Path.push_front(temp_Node);
						parentid = temp_Node->parent_id;
					}
					int numofsamples = RRT_Path.size();
					mexPrintf("Size is %d\n",numofsamples);
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

				else{
					continue;
				}				
				//rrt.print_Vertices();
				//rrt.print_Edges();

			}

			else{
				for (int k =0; k<numofDOFs; ++k){
					current_eps_step_angles[k] = nearest_node_angles[k] + epsilon*arm_movement_sign[k]*difference_angles[k];
				}
				// collision check added here
				if(IsValidArmConfiguration(current_eps_step_angles, numofDOFs, map, x_size, y_size)){
					cur_node_id = rrt.add_Vertex_ret_id(current_eps_step_angles[0], current_eps_step_angles[1], current_eps_step_angles[2], current_eps_step_angles[3], current_eps_step_angles[4], nearest_node_id);
					rrt.add_Edge(nearest_node_id, cur_node_id);
				}
				else{
					//mexPrintf("Collision \n");
					continue;
				}
			}

		}
		else{
			for (int j =0; j<numofDOFs; ++j){
				current_sample_angles[j] = 2*PI*uni_distribution(generator);
			}
			nearest_node_id = rrt.getNearestNeighbour(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
			convert_to_unit(difference_angles, numofDOFs);

			if (nearest_dist < epsilon){
				// collision check added here
				if(IsValidArmConfiguration(current_sample_angles, numofDOFs, map, x_size, y_size)){
					cur_node_id = rrt.add_Vertex_ret_id(current_sample_angles[0], current_sample_angles[1], current_sample_angles[2], current_sample_angles[3], current_sample_angles[4], nearest_node_id);			
					rrt.add_Edge(nearest_node_id, cur_node_id);
					continue;
				}
				else{
					//mexPrintf("Collision \n");
					continue;
				}

			}

			else{
				for (int k =0; k<numofDOFs; ++k){
					current_eps_step_angles[k] = nearest_node_angles[k] + epsilon*arm_movement_sign[k]*difference_angles[k];
				}
			// collision check added here
				if(IsValidArmConfiguration(current_eps_step_angles, numofDOFs, map, x_size, y_size)){
					cur_node_id = rrt.add_Vertex_ret_id(current_eps_step_angles[0], current_eps_step_angles[1], current_eps_step_angles[2], current_eps_step_angles[3], current_eps_step_angles[4], nearest_node_id);
					rrt.add_Edge(nearest_node_id, cur_node_id);
				}
				else{
					//mexPrintf("Collision \n");
					continue;
				}
			}
		}	
	}

	//rrt.print_Vertices();
	mexPrintf("Did not find a path \n");
	return;
}