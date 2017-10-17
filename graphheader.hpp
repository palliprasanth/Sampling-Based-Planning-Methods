#ifndef GRAPH_HEADER_H   
#define GRAPH_HEADER_H

using namespace std;

typedef struct Node Node;
struct Node{
	int node_id;
	int parent_id;
	double theta_1;
	double theta_2;
	double theta_3;
	double theta_4;
	double theta_5;
};

typedef struct Edge Edge;
struct Edge{
	int node1_id;
	int node2_id;
};


class Graph{
private:
	list<Node> Vertices;
	list<Edge> Edges;
	int counter;
public:
	Graph();
	list<Node>* get_Vertices();
	list<Edge>* get_Edges();
	Node* get_Vertex(int);
	void add_Vertex(double, double, double, double, double);
	int add_Vertex_ret_id(double, double, double, double, double, int);
	void add_Edge(int, int);
	void delete_Vertex(int);
	int getNearestNeighbour(Node*);
	int getNearestNeighbour(double*, double*, int*, double*, double*);
	void print_Vertices();
	void print_Edges();
	~Graph();		
};


#endif 