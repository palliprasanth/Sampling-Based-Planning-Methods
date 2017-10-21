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

#define EPS 0.3


# define VOL_HYPER 5.264
# define MAX_NEIGHBOR EPS
# define GAMMA 1	


int cur_node_id, nearest_node_id;
Node* nearest_node_address;
Node* cur_Node;
double nearest_dist, neigh_dist;
double current_eps_step_angles[5], nearest_node_angles[5], difference_angles[5], adding_angles[5];
int arm_movement_sign[5]; // To keep track of nearest motion to be clockwise or anti-clockwise
int obstacle_check;
double temp_distance;
double temp_cost, min_cost;
double distance_to_neighbor;

vector<int> connect_rrt(Graph* rrt_tree, double* map, int x_size, int y_size, double* current_sample_angles, int numofDOFs){
	vector<int> connect_return = {0,1,1};
	while (connect_return[0] == 0){
		//mexPrintf("Entered Connect RRT\n");
		connect_return = extend_rrt(rrt_tree, map, x_size, y_size, current_sample_angles, numofDOFs);
		//mexPrintf("Connect Track : %d\n", connect_return[0]);
	}
	return connect_return;
}

vector<int> extend_rrt(Graph* rrt_tree, double* map, int x_size, int y_size, double* current_sample_angles, int numofDOFs){
	/*
	For first index element
	Trapped = -1
	Advanced = 0
	Reached = 1
	*/
	vector<int> extend_return = {-1,1,1};
	nearest_node_id = rrt_tree->getNearestNeighbour(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
	convert_to_unit(difference_angles, numofDOFs);

	if (nearest_dist < EPS){
		obstacle_check = IsValidArmMovement(nearest_node_angles, adding_angles, difference_angles, nearest_dist, arm_movement_sign, numofDOFs, map, x_size, y_size);
		if(obstacle_check == 1){
			cur_node_id = rrt_tree->add_Vertex_ret_id(adding_angles[0], adding_angles[1], adding_angles[2], adding_angles[3], adding_angles[4], nearest_node_id);			
			//rrt_tree->add_Edge(nearest_node_id, cur_node_id);
			extend_return[0] = 1;
			extend_return[1] = nearest_node_id;
			extend_return[2] = cur_node_id;
			return extend_return;
		}
		else if(obstacle_check == 0){
			cur_node_id = rrt_tree->add_Vertex_ret_id(adding_angles[0], adding_angles[1], adding_angles[2], adding_angles[3], current_sample_angles[4], nearest_node_id);
			extend_return[0] = 0;
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

		obstacle_check = IsValidArmMovement(nearest_node_angles, current_eps_step_angles, difference_angles, EPS, arm_movement_sign, numofDOFs, map, x_size, y_size);
		if(obstacle_check != -1){
			cur_node_id = rrt_tree->add_Vertex_ret_id(current_eps_step_angles[0], current_eps_step_angles[1], current_eps_step_angles[2], current_eps_step_angles[3], current_eps_step_angles[4], nearest_node_id);
			extend_return[0] = 0;
			extend_return[1] = nearest_node_id;
			extend_return[2] = cur_node_id;
			return extend_return;
		}
		else{
			extend_return[0] = -1;
			return extend_return;
		}
	}
}

vector<int> extend_rrt_star(Graph* rrt_tree, double* map, int x_size, int y_size, double* current_sample_angles, int numofDOFs){
	/*
	For first index element
	Trapped = -1
	Advanced = 0
	Reached = 1
	*/
	vector<int> extend_return = {-1,1,1};
	Node* x_min;
	nearest_node_address = rrt_tree->getNearestNeighbourAddress(current_sample_angles, &nearest_dist, arm_movement_sign, nearest_node_angles, difference_angles);
	x_min = nearest_node_address;
	convert_to_unit(difference_angles, numofDOFs);

	if (nearest_dist < EPS){
		obstacle_check = IsValidArmMovement(nearest_node_angles, adding_angles, difference_angles, nearest_dist, arm_movement_sign, numofDOFs, map, x_size, y_size);
		if(obstacle_check == 1){

			cur_node_id = rrt_tree->add_Vertex_with_cost(adding_angles[0], adding_angles[1], adding_angles[2], adding_angles[3], adding_angles[4], nearest_node_address->node_id, nearest_node_address->cost + nearest_dist);			
			cur_Node = rrt_tree->get_Vertex(cur_node_id);
			// mexPrintf("Node Id: %d\n",cur_Node->node_id);
			// mexPrintf("Parent: %d\n", cur_Node->parent_id);	
			//rrt_tree->add_Edge(nearest_node_id, cur_node_id);
			extend_return[0] = 1;
			extend_return[1] = cur_Node->parent_id;
			extend_return[2] = cur_Node->node_id;
			return extend_return;
		}
		else if(obstacle_check == 0){
			// Do rewiring here
			temp_distance = get_distance_angular(nearest_node_angles, adding_angles);
			cur_node_id = rrt_tree->add_Vertex_with_cost(adding_angles[0], adding_angles[1], adding_angles[2], adding_angles[3], current_sample_angles[4], nearest_node_address->node_id, nearest_node_address->cost + temp_distance);
			cur_Node = rrt_tree->get_Vertex(cur_node_id);

			neigh_dist = get_neighbourhood_distance(rrt_tree->get_number_vertices());
			list<Node*> Neighbourhood_V;
			rrt_tree->getNearestNeighboursinNeighbourhood(adding_angles, cur_Node->node_id, &Neighbourhood_V, neigh_dist);
			
			int list_size = Neighbourhood_V.size();
			min_cost = cur_Node->cost;

			for (list<Node*>::iterator it = Neighbourhood_V.begin(); it != Neighbourhood_V.end(); it++){
				
				int obstacle_check_2 = IsValidArmMovement(*it, cur_Node, &distance_to_neighbor, numofDOFs, map, x_size, y_size);

				if (obstacle_check_2){
					temp_cost = (*it)->cost + distance_to_neighbor; 
					if (temp_cost < min_cost){
						x_min = (*it);
						min_cost = temp_cost;
					}
				}
			}

			cur_Node->parent_id =  x_min->node_id;
			int node_id_x_min = x_min->node_id;
			cur_Node->cost = min_cost;
			rrt_tree->propagate_costs(cur_Node);
			rrt_tree->add_Edge(x_min, cur_Node);	
			//mexPrintf("\n");

			for (list<Node*>::iterator it = Neighbourhood_V.begin(); it != Neighbourhood_V.end(); it++){
				if ((*it)->node_id != node_id_x_min){
					int obstacle_check_3 = IsValidArmMovement(cur_Node, *it, &distance_to_neighbor, numofDOFs, map, x_size, y_size);
					if (obstacle_check_3){
						if ((cur_Node->cost + distance_to_neighbor) < (*it)->cost){

							(*it)->parent_id = cur_Node->node_id;
							(*it)->cost = cur_Node->cost + distance_to_neighbor;
							rrt_tree->propagate_costs(*it);
							mexPrintf("Second Rewiring \n");
						}
					}
				}
			}

			extend_return[0] = 0;
			extend_return[1] = cur_Node->parent_id;
			extend_return[2] = cur_Node->node_id;
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

		obstacle_check = IsValidArmMovement(nearest_node_angles, current_eps_step_angles, difference_angles, EPS, arm_movement_sign, numofDOFs, map, x_size, y_size);
		if(obstacle_check != -1){
			temp_distance = get_distance_angular(nearest_node_angles, current_eps_step_angles);
			cur_node_id = rrt_tree->add_Vertex_with_cost(current_eps_step_angles[0], current_eps_step_angles[1], current_eps_step_angles[2], current_eps_step_angles[3], current_eps_step_angles[4], nearest_node_address->node_id, nearest_node_address->cost + temp_distance);
			cur_Node = rrt_tree->get_Vertex(cur_node_id);
			neigh_dist = get_neighbourhood_distance(rrt_tree->get_number_vertices());
			list<Node*> Neighbourhood_V;
			rrt_tree->getNearestNeighboursinNeighbourhood(current_eps_step_angles, cur_Node->node_id, &Neighbourhood_V, neigh_dist);
			
			int list_size = Neighbourhood_V.size();
			min_cost = cur_Node->cost;
			// mexPrintf("Printing Neighbourhood\n");
			// mexPrintf("Cur Node id: %d\n", cur_node_id);
			for (list<Node*>::iterator it = Neighbourhood_V.begin(); it != Neighbourhood_V.end(); it++){
				
				int obstacle_check_2 = IsValidArmMovement(*it, cur_Node, &distance_to_neighbor, numofDOFs, map, x_size, y_size);

				if (obstacle_check_2){
					temp_cost = (*it)->cost + distance_to_neighbor; 
					if (temp_cost < min_cost){
						x_min = (*it);
						min_cost = temp_cost;
						//mexPrintf("First Rewiring\n");
					}
				}
			}

			cur_Node->parent_id =  x_min->node_id;
			int node_id_x_min = x_min->node_id;
			cur_Node->cost = min_cost;
			rrt_tree->propagate_costs(cur_Node);
			rrt_tree->add_Edge(x_min, cur_Node);	
			//mexPrintf("\n");

			for (list<Node*>::iterator it = Neighbourhood_V.begin(); it != Neighbourhood_V.end(); it++){
				if ((*it)->node_id != node_id_x_min){
					int obstacle_check_3 = IsValidArmMovement(cur_Node, *it, &distance_to_neighbor, numofDOFs, map, x_size, y_size);
			 		// mexPrintf("Cur Node Id: %d\n",cur_Node->node_id);
			 		// mexPrintf("Cur Node Cost: %f\n",cur_Node->cost);
			 		// mexPrintf("Cur Neighbour Id: %d\n",(*it)->node_id);
			 		// mexPrintf("Cur Neighbour Cost: %f\n",(*it)->cost);
			 		// mexPrintf("Dist to Neighbour: %f\n",distance_to_neighbor);
			 		// mexPrintf("\n");
					if (obstacle_check_3){
						if ((cur_Node->cost + distance_to_neighbor) < (*it)->cost){

							(*it)->parent_id = cur_Node->node_id;
							(*it)->cost = cur_Node->cost + distance_to_neighbor;
							rrt_tree->propagate_costs(*it);
							mexPrintf("Second Rewiring \n");
						}
					}
				}
			}

			extend_return[0] = 0;
			extend_return[1] = cur_Node->parent_id;
			extend_return[2] = cur_Node->node_id;
			return extend_return;
		}
		else{
			extend_return[0] = -1;
			return extend_return;
		}


	}

}


void build_rrt(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength){

	mexPrintf("Entered RRT \n");

	const int max_iter = 20000;
	double bias_random;
	vector<int> extend_track;
	double current_sample_angles[5];

	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Graph rrt;
	rrt.add_Vertex(armstart_anglesV_rad[0], armstart_anglesV_rad[1], armstart_anglesV_rad[2], armstart_anglesV_rad[3], armstart_anglesV_rad[4]);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> bias_distribution(0.0,1.2);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);
	int num_vertices = 0;
	while (num_vertices <= max_iter){
		num_vertices = rrt.get_number_vertices();
		bias_random = bias_distribution(generator);
		if (bias_random > 1){
			extend_track = extend_rrt(&rrt, map, x_size, y_size, armgoal_anglesV_rad, numofDOFs);
			if (extend_track[0] == 1){
				mexPrintf("Found a path with expansion of %d states \n",num_vertices);

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
	mexPrintf("Did not find a path after expansion of %d states\n", max_iter);
	return;
}


void build_rrt_connect(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength){

	mexPrintf("Entered RRT Connect\n");

	const int max_iter = 20000;
	vector<int> extend_track;
	vector<int> connect_track;
	double current_sample_angles[5], new_sample_angles[5];
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Graph rrtA, rrtB;
	rrtA.add_Vertex(armstart_anglesV_rad[0], armstart_anglesV_rad[1], armstart_anglesV_rad[2], armstart_anglesV_rad[3], armstart_anglesV_rad[4]);
	rrtB.add_Vertex(armgoal_anglesV_rad[0], armgoal_anglesV_rad[1], armgoal_anglesV_rad[2], armgoal_anglesV_rad[3], armgoal_anglesV_rad[4]);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);

	Graph* Current_Tree;
	Graph* Other_Tree;
	Current_Tree = &rrtA;
	Other_Tree = &rrtB;

	int num_vertices = 0;
	while (num_vertices <= max_iter){
		num_vertices = rrtA.get_number_vertices() + rrtB.get_number_vertices();
		for (int j =0; j<numofDOFs; ++j){
			current_sample_angles[j] = 2*PI*uni_distribution(generator);
		}
		//mexPrintf("Extending Trees \n");	
		extend_track = extend_rrt(Current_Tree, map, x_size, y_size, current_sample_angles, numofDOFs);
		//mexPrintf("Extend Track : %d\n", extend_track[0]);
		if (extend_track[0] != -1){
			int new_node_id = extend_track[2];
			Node* temp_Node = Current_Tree->get_Vertex(new_node_id);
			new_sample_angles[0] = temp_Node->theta_1;
			new_sample_angles[1] = temp_Node->theta_2;
			new_sample_angles[2] = temp_Node->theta_3;
			new_sample_angles[3] = temp_Node->theta_4;
			new_sample_angles[4] = temp_Node->theta_5;
			//mexPrintf("Connecting Trees \n");
			connect_track = connect_rrt(Other_Tree, map, x_size, y_size, new_sample_angles, numofDOFs);
			if(connect_track[0] == 1){
				mexPrintf("Found a path with expansion of %d states \n",num_vertices);
				int parentid = rrtA.get_Vertices()->back().parent_id;
				list<Node*> Start_Tree;
				Node* temp_Node;
				while (parentid != 0){
					temp_Node = rrtA.get_Vertex(parentid);
					Start_Tree.push_front(temp_Node);
					parentid = temp_Node->parent_id;
				}

				parentid = rrtB.get_Vertices()->back().parent_id;
				temp_Node = rrtB.get_Vertex(parentid);
				parentid = temp_Node->parent_id;

				while (parentid != 0){
					temp_Node = rrtB.get_Vertex(parentid);
					Start_Tree.push_back(temp_Node);
					parentid = temp_Node->parent_id;
				}
				int numofsamples = Start_Tree.size();
				mexPrintf("Number of steps: %d\n", numofsamples);
				*plan = (double**) malloc(numofsamples*sizeof(double*));
				int p = 0;
				for (list<Node*>::iterator it = Start_Tree.begin(); it != Start_Tree.end(); it++){
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
		// Swapping Trees
		Graph* temp_graph;
		temp_graph = Current_Tree;
		Current_Tree = Other_Tree;
		Other_Tree = temp_graph;
	}
	mexPrintf("Did not find a path after expansion of %d states\n", max_iter);
	return;
}

void build_rrt_star(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength){

	mexPrintf("Entered RRT Star\n");
	double bias_random;
	vector<int> extend_track;
	const int max_iter = 20000;
	double current_sample_angles[5];

	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Graph rrt_star;
	rrt_star.add_Vertex(armstart_anglesV_rad[0], armstart_anglesV_rad[1], armstart_anglesV_rad[2], armstart_anglesV_rad[3], armstart_anglesV_rad[4]);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> bias_distribution(0.0,1.2);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);
	int num_vertices = 0;
	
	while (num_vertices <= max_iter){
		num_vertices = rrt_star.get_number_vertices();
		bias_random = bias_distribution(generator);
		if (bias_random > 1){
			extend_track = extend_rrt_star(&rrt_star, map, x_size, y_size, armgoal_anglesV_rad, numofDOFs);
			if (extend_track[0] == 1){
				mexPrintf("Found a path with expansion %d \n",extend_track[2]);

				int parentid = extend_track[1];
				list<Node*> RRT_Path;
				Node* temp_Node;
				while (parentid != 0){
					temp_Node = rrt_star.get_Vertex(parentid);
					RRT_Path.push_front(temp_Node);
					parentid = temp_Node->parent_id;
				}
				int numofsamples = RRT_Path.size();
				*plan = (double**) malloc(numofsamples*sizeof(double*));
				mexPrintf("Size of Path: %d\n",numofsamples);
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
			extend_track = extend_rrt_star(&rrt_star, map, x_size, y_size, current_sample_angles, numofDOFs);
		}	
	}
	//rrt_star.print_Vertices();
	mexPrintf("Did not find a path after expansion of %d states\n", max_iter);
	return;
}


void build_prm(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
	int numofDOFs, double*** plan, int* planlength){

	mexPrintf("Entered Probability Road Map\n");

	return;
}
