#ifndef GRAPH_HEADER_H   
#define GRAPH_HEADER_H

using namespace std;

typedef struct Node Node;
typedef struct Edge Edge;


struct Node{
	int node_id;
	int parent_id;
	Node* parent;
	double cost;
	bool is_visited;
	double theta_1;
	double theta_2;
	double theta_3;
	double theta_4;
	double theta_5;
	list<Node*> children;
	list<Edge*> edges;
	bool is_closed;

	bool operator<(const Node& node_2) const
	{
		return cost < node_2.cost;
	}
};


struct Edge{
	Node* vertex1;
	Node* vertex2;
	double cost;
};


class Tree{
protected:
	list<Node> Vertices;
	//list<Edge> Edges;
	int counter;
	int number_of_vertices;
public:
	Tree();
	list<Node>* get_Vertices();
	//list<Edge>* get_Edges();
	int get_number_vertices();
	Node* get_Vertex(int);
	void add_Vertex(double, double, double, double, double);
	int add_Vertex_with_cost(double, double, double, double, double, int, double);
	int add_Vertex_ret_id(double, double, double, double, double, int);
	//void add_Edge(int, int);
	void add_Edge(Node*, Node*);
	void delete_Vertex(int);
	void delete_Edge(Node*, Node*);
	int getNearestNeighbour(Node*);
	void getNearestNeighboursinNeighbourhood(double*, int, list<Node*>*, double);
	int getNearestNeighbour(double*, double*, int*, double*, double*);
	Node* getNearestNeighbourAddress(double*, double*, int*, double*, double*);
	void propagate_costs(Node*);
	void print_Vertices();
	//void print_Edges();
	~Tree();		
};


class Graph : public Tree{
private:
	list<Edge> Edges;
public:
	list<Edge>* get_Edges();
	Node* add_Vertex(double, double, double, double, double);
	void add_Edge(Node*, Node*, double);
	void print_Edges();
};


#endif 