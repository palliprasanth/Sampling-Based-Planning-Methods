#include <math.h>
#include "mex.h"
#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include "graphheader.hpp"
#include "plannerheader.hpp"
#include "constants.hpp"
#include <iterator>
using namespace std;

Tree::Tree(){
	counter = 1;
	number_of_vertices = 0;
	//mexPrintf("Tree Created\n");
}

list<Node>* Tree::get_Vertices(){
	return &this->Vertices;
}

int Tree::get_number_vertices(){
	return this->number_of_vertices;
}

Node* Tree::get_Vertex(int nodeid){
	Node* temp;	
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		if (it->node_id == nodeid){
			temp =  &(*it);
			return temp;
		}
	}
}

void Tree::add_Vertex(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;
	temp_node.is_closed = false;
	temp_node.is_visited = false;
	temp_node.parent_id = 0;
	temp_node.cost = 0;
	Vertices.push_back(temp_node);

	counter++;
	number_of_vertices++;
}

int Tree::add_Vertex_with_cost(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, int parentid, double cost){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;
	temp_node.parent_id = parentid;
	temp_node.is_closed = false;
	temp_node.is_visited = false;
	temp_node.cost = cost;
	Vertices.push_back(temp_node);

	counter++;
	number_of_vertices++;
	return temp_node.node_id;
}

void Tree::add_Edge(Node* parent, Node* child){
	parent->children.push_back(child);
}

int Tree::add_Vertex_ret_id(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, int parentid){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;
	temp_node.parent_id = parentid;
	temp_node.is_closed = false;
	temp_node.is_visited = false;
	Vertices.push_back(temp_node);

	counter++;
	number_of_vertices++;
	return temp_node.node_id;
}


// void Graph::add_Edge(int nodeid1, int nodeid2){
// 	Edge temp_edge;
// 	temp_edge.node1_id = nodeid1;
// 	temp_edge.node2_id = nodeid2;

// 	Edges.push_back(temp_edge);
// }

void Tree::delete_Vertex(int nodeid){
	// Also deletes the edges corresponding to that nodeid
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		if (it->node_id == nodeid){
			Vertices.erase(it);
			break;
		}
	}
	number_of_vertices--;
	// for (list<Edge>::iterator it = Edges.begin(); it != Edges.end(); it++){
	// 	if (it->node1_id == nodeid || it->node2_id == nodeid){
	// 		Edges.erase(it);
	// 		continue;
	// 	}
	// }

}

int Tree::getNearestNeighbour(Node* cur_Node){
	Node* temp;
	double min_dist = 10000.0;
	int min_id = -1;
	double distance;
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		temp = &(*it);
		distance = get_distance_angular(temp, cur_Node); 
		if (distance < min_dist){
			min_dist = distance;
			min_id = temp->node_id;
		}		
	}
	return min_id;
}

int Tree::getNearestNeighbour(double* Final_Node_angles, double* min_distance, int* movement_sign, double* cur_node_angles, double* difference_angles){
	Node* temp;
	*min_distance = 10000.0;
	int move_sign[5];
	int min_id = -1;
	double distance;
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		temp = &(*it);
		distance = get_distance_angular(temp, Final_Node_angles, move_sign, difference_angles); 
		if (distance < *min_distance){
			*min_distance = distance;
			for (int i=0; i<5; ++i){
				movement_sign[i] = move_sign[i];
			}
			min_id = temp->node_id;
			cur_node_angles[0] = temp->theta_1;
			cur_node_angles[1] = temp->theta_2;
			cur_node_angles[2] = temp->theta_3;
			cur_node_angles[3] = temp->theta_4;
			cur_node_angles[4] = temp->theta_5;
		}		
	}
	return min_id;
}


Node* Tree::getNearestNeighbourAddress(double* Final_Node_angles, double* min_distance, int* movement_sign, double* cur_node_angles, double* difference_angles){
	Node* temp;
	Node* temp2;
	*min_distance = 10000.0;
	int move_sign[5];
	int min_id = -1;
	double distance;
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		temp = &(*it);
		distance = get_distance_angular(temp, Final_Node_angles, move_sign, difference_angles); 
		if (distance < *min_distance){
			*min_distance = distance;
			for (int i=0; i<5; ++i){
				movement_sign[i] = move_sign[i];
			}
			temp2 = temp;
			cur_node_angles[0] = temp->theta_1;
			cur_node_angles[1] = temp->theta_2;
			cur_node_angles[2] = temp->theta_3;
			cur_node_angles[3] = temp->theta_4;
			cur_node_angles[4] = temp->theta_5;
		}		
	}
	return temp2;
}


void Tree::getNearestNeighboursinNeighbourhood(double* cur_Node, int cur_id, list<Node*>* Neighbourhood, double neighbour_distance){
	Node* temp;
	double min_dist = 10000.0;
	int min_id = -1;
	double distance;
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		temp = &(*it);
		distance = get_distance_angular(temp, cur_Node); 
		if (distance < neighbour_distance){
			if (temp->node_id!=cur_id)
				Neighbourhood->push_back(temp);
		}		
	}

}

void Tree::delete_Edge(Node* Parent_node, Node* Child_Node){

	int nodeid = Child_Node->node_id;

	for (list<Node*>::iterator it = Parent_node->children.begin(); it != Parent_node->children.end(); it++){
		if ((*it)->node_id == nodeid){
			Parent_node->children.erase(it);
			return;
		}
	}
}

void Tree::propagate_costs(Node* Parent_node){

	// int num_children = Parent_node->children.size();
	// //mexPrintf("Propagating Cost\n");
	// if (num_children == 0){
	// 	return;
	// }
	// else{
		for (list<Node*>::iterator it = Parent_node->children.begin(); it != Parent_node->children.end(); it++){
			double distance = get_distance_angular(Parent_node, *it);
			(*it)->cost = Parent_node->cost + distance;
			this->propagate_costs(*it);
		}
	// }

	return;

}

void Tree::print_Vertices(){
	mexPrintf("Printing Vertices\n");
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		mexPrintf("node_id = %d\n",it->node_id);
		mexPrintf("parent_id = %d\n",it->parent_id);
		mexPrintf("Cost = %f\n",it->cost);
		mexPrintf("Number of children: %d\n",it->children.size());
		mexPrintf("q1 = %f\n",it->theta_1);
		mexPrintf("q2 = %f\n",it->theta_2);
		mexPrintf("q3 = %f\n",it->theta_3);
		mexPrintf("q4 = %f\n",it->theta_4);
		mexPrintf("q5 = %f\n",it->theta_5);
		mexPrintf("\n");
	}
}

Tree::~Tree(){
	//mexPrintf("Tree Destroyed\n");
}


list<Edge>* Graph::get_Edges(){
	return &this->Edges;
}

Node* Graph::add_Vertex(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;
	temp_node.parent_id = 0;
	temp_node.cost = 0;
	temp_node.is_closed = false;
	temp_node.is_visited = false;
	Vertices.push_back(temp_node);

	counter++;
	number_of_vertices++;
	return &Vertices.back();
}

void Graph::add_Edge(Node* Vertex1, Node* Vertex2, double edge_cost){
	Edge temp_edge;
	temp_edge.vertex1 = Vertex1;
	temp_edge.vertex2 = Vertex2;
	temp_edge.cost = edge_cost;

	Edges.push_back(temp_edge);
	Edge* temp_edge_ptr = &Edges.back();
	Vertex1->edges.push_back(temp_edge_ptr);
	Vertex2->edges.push_back(temp_edge_ptr);
}

void Graph::print_Edges(){
	//mexPrintf("Printing Edges\n");
	mexPrintf("Number of Edges: %d\n",Edges.size());
	// for (list<Edge>::iterator it = Edges.begin(); it != Edges.end(); it++){
	// 	mexPrintf("Edge between %d and %d with cost : %f\n",it->vertex1->node_id,it->vertex2->node_id, it->cost);	
	// }
	// mexPrintf("\n");
}
