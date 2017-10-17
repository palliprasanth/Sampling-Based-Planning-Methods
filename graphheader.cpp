#include <math.h>
#include "mex.h"
#include <iostream>
#include <vector>
#include <list>
#include "graphheader.hpp"
#include "plannerheader.hpp"
#include <iterator>
using namespace std;

Graph::Graph(){
	counter = 1;
	mexPrintf("Graph Created\n");
}

list<Node>* Graph::get_Vertices(){
	return &this->Vertices;
}

list<Edge>* Graph::get_Edges(){
	return &this->Edges;
}

Node* Graph::get_Vertex(int nodeid){
	Node* temp;	
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		if (it->node_id == nodeid){
			temp =  &(*it);
			return temp;
		}
	}
}

void Graph::add_Vertex(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;

	Vertices.push_back(temp_node);

	counter++;
}

int Graph::add_Vertex_ret_id(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, int parentid){
	Node temp_node;
	temp_node.node_id = counter;
	temp_node.theta_1 = theta_1;
	temp_node.theta_2 = theta_2;
	temp_node.theta_3 = theta_3;
	temp_node.theta_4 = theta_4;
	temp_node.theta_5 = theta_5;
	temp_node.parent_id = parentid;
	Vertices.push_back(temp_node);

	counter++;

	return temp_node.node_id;
}


void Graph::add_Edge(int nodeid1, int nodeid2){
	Edge temp_edge;
	temp_edge.node1_id = nodeid1;
	temp_edge.node2_id = nodeid2;

	Edges.push_back(temp_edge);
}

void Graph::delete_Vertex(int nodeid){
	// Also deletes the edges corresponding to that nodeid
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		if (it->node_id == nodeid){
			Vertices.erase(it);
			break;
		}
	}
	for (list<Edge>::iterator it = Edges.begin(); it != Edges.end(); it++){
		if (it->node1_id == nodeid || it->node2_id == nodeid){
			Edges.erase(it);
			continue;
		}
	}

}

int Graph::getNearestNeighbour(Node* cur_Node){
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

int Graph::getNearestNeighbour(double* cur_Node_angles, double* min_distance, int* movement_sign){
	Node* temp;
	*min_distance = 10000.0;
	int move_sign[5];
	int min_id = -1;
	double distance;
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		temp = &(*it);
		distance = get_distance_angular(temp, cur_Node_angles, move_sign); 
		if (distance < *min_distance){
			*min_distance = distance;
			movement_sign = move_sign;
			min_id = temp->node_id;
		}		
	}
	return min_id;
}


void Graph::print_Vertices(){
	mexPrintf("Printing Vertices\n");
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		mexPrintf("node_id = %d\n",it->node_id);
		mexPrintf("q1 = %f\n",it->theta_1);
		mexPrintf("q2 = %f\n",it->theta_2);
		mexPrintf("q3 = %f\n",it->theta_3);
		mexPrintf("q4 = %f\n",it->theta_4);
		mexPrintf("q5 = %f\n",it->theta_5);
		mexPrintf("\n");
	}
}

void Graph::print_Edges(){
	mexPrintf("Printing Edges\n");
	for (list<Edge>::iterator it = Edges.begin(); it != Edges.end(); it++){
		mexPrintf("Edge between %d and %d\n",it->node1_id,it->node2_id);	
	}
	mexPrintf("\n");
}

Graph::~Graph(){
	mexPrintf("Graph Destroyed\n");
}