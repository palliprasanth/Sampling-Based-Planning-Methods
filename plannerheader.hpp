#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

// Already defined Functions
typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);
void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
void get_current_point(bresenham_param_t *params, int *x, int *y);
int get_next_point(bresenham_param_t*);
int IsValidLineSegment(double, double, double, double, double*, int, int);
int IsValidArmConfiguration(double*, int, double*, int, int);

// Functions defined by me
double deg2rad(double);
double rad2deg(double);
double angle_between(double, double);
double angle_between(double, double, int*);
double get_distance_angular(Node*, Node*);
double get_distance_angular(Node*, double*, int*, double*);
void convert_to_unit(double* ,int);


vector<int> extend_rrt(Graph* , double* , int , int , double* , int );
void build_rrt(double*, int, int, double*, double*, int, double***, int*, int*);
void build_rrt_connect(double*, int, int, double*, double*, int, double***, int*, int*);
#endif 