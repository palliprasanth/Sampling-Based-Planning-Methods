#ifndef GRAPH_HEADER_H   
#define GRAPH_HEADER_H

using namespace std;

typedef struct Node Node;
typedef struct Edge Edge;


struct Node{
	int node_id;
	int parent_id;
	double cost;
	double theta_1;
	double theta_2;
	double theta_3;
	double theta_4;
	double theta_5;
};


struct Edge{
	int node1_id;
	int node2_id;
};


class Graph{
private:
	list<Node> Vertices;
	//list<Edge> Edges;
	int counter;
	int number_of_vertices;
public:
	Graph();
	list<Node>* get_Vertices();
	list<Edge>* get_Edges();
	int get_number_vertices();
	Node* get_Vertex(int);
	void add_Vertex(double, double, double, double, double);
	void add_Vertex_with_cost(double, double, double, double, double, int, double);
	int add_Vertex_ret_id(double, double, double, double, double, int);
	//void add_Edge(int, int);
	void delete_Vertex(int);
	int getNearestNeighbour(Node*);
	void getNearestNeighboursinNeighbourhood(Node*, list<Node*>, double*);
	int getNearestNeighbour(double*, double*, int*, double*, double*);
	void print_Vertices();
	//void print_Edges();
	~Graph();		
};


#endif 