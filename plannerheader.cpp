#include <math.h>
#include "mex.h"
#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <random>
#include <iterator>
#include <ctime>
#include <chrono>
#include "graphheader.hpp"
#include "plannerheader.hpp"
#include "constants.hpp"

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
	{
		params->Y1=p1x;
		params->X1=p1y;
		params->Y2=p2x;
		params->X2=p2y;
	}
	else
	{
		params->X1=p1x;
		params->Y1=p1y;
		params->X2=p2x;
		params->Y2=p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0)
	{
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
			*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
			*y = -*y;
	}
}

int get_next_point(bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
	int x_size,
	int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}


int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
	int x_size, int y_size)
{
	double x0,y0,x1,y1;
	int i;

 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
}


double deg2rad(double input){
	return (input*PI)/180.0;
}

double rad2deg(double input){
	return (input*180.0)/PI;
}

void wrap_to_2pi(double* input){
	while(*input < 0 || *input >= 2*PI){
		if (*input < 0)
			*input += 2*PI;
		else
			*input -= 2*PI;
	}
}


double angle_between(double angle1, double angle2, int* move_sign){
	double difference1 = angle1 - angle2;
	double difference2 = angle2 - angle1;
	*move_sign = 1;
	if (difference1 < 0)
		difference1 = difference1 + 2*PI;

	if (difference2 < 0)
		difference2 = difference2 + 2*PI;

	if (difference1 <= difference2){
		*move_sign = -1;
		return difference1;
	}
	else
		return difference2;
}

double angle_between(double angle1, double angle2){
	double difference1 = angle1 - angle2;
	double difference2 = angle2 - angle1;
	if (difference1 < 0)
		difference1 = difference1 + 2*PI;

	if (difference2 < 0)
		difference2 = difference2 + 2*PI;

	if (difference1 <= difference2)
		return difference1;
	else
		return difference2;
}

double get_distance_angular(Node* node1, Node* node2){
	double distance = 0;

	distance = pow(angle_between(node1->theta_1,node2->theta_1),2.0) + pow(angle_between(node1->theta_2,node2->theta_2),2.0) + pow(angle_between(node1->theta_3,node2->theta_3),2.0) + pow(angle_between(node1->theta_4,node2->theta_4),2.0) + pow(angle_between(node1->theta_5,node2->theta_5),2.0);
	distance = sqrt(distance);
	return distance;

}



double get_distance_angular(Node* node1, Node* node2, int* move_sign, double* difference_angles){
	double distance = 0;
	difference_angles[0] = angle_between(node1->theta_1,node2->theta_1, &move_sign[0]);
	difference_angles[1] = angle_between(node1->theta_2,node2->theta_2, &move_sign[1]);
	difference_angles[2] = angle_between(node1->theta_3,node2->theta_3, &move_sign[2]);
	difference_angles[3] = angle_between(node1->theta_4,node2->theta_4, &move_sign[3]);
	difference_angles[4] = angle_between(node1->theta_5,node2->theta_5, &move_sign[4]);
	for (int i=0;i<5;i++){
		distance += pow(difference_angles[i],2.0);
	}

	distance = sqrt(distance);
	return distance;

}


double get_distance_angular(Node* node1, double* node2_angle, int* move_sign, double* difference_angles){
	double distance = 0;
	difference_angles[0] = angle_between(node1->theta_1,node2_angle[0], &move_sign[0]);
	difference_angles[1] = angle_between(node1->theta_2,node2_angle[1], &move_sign[1]);
	difference_angles[2] = angle_between(node1->theta_3,node2_angle[2], &move_sign[2]);
	difference_angles[3] = angle_between(node1->theta_4,node2_angle[3], &move_sign[3]);
	difference_angles[4] = angle_between(node1->theta_5,node2_angle[4], &move_sign[4]);
	for (int i=0;i<5;i++){
		distance += pow(difference_angles[i],2.0);
	}

	distance = sqrt(distance);
	return distance;

}

double get_distance_angular(Node* node1, double* node2_angle){
	double distance = 0;
	double temp_angles[5];
	temp_angles[0] = angle_between(node1->theta_1,node2_angle[0]);
	temp_angles[1] = angle_between(node1->theta_2,node2_angle[1]);
	temp_angles[2] = angle_between(node1->theta_3,node2_angle[2]);
	temp_angles[3] = angle_between(node1->theta_4,node2_angle[3]);
	temp_angles[4] = angle_between(node1->theta_5,node2_angle[4]);
	for (int i=0;i<5;i++){
		distance += pow(temp_angles[i],2.0);
	}

	distance = sqrt(distance);
	return distance;

}

double get_distance_angular(double* node1_angle, double* node2_angle){
	double distance = 0;
	double temp_angles[5];
	temp_angles[0] = angle_between(node1_angle[0],node2_angle[0]);
	temp_angles[1] = angle_between(node1_angle[1],node2_angle[1]);
	temp_angles[2] = angle_between(node1_angle[2],node2_angle[2]);
	temp_angles[3] = angle_between(node1_angle[3],node2_angle[3]);
	temp_angles[4] = angle_between(node1_angle[4],node2_angle[4]);
	//distance = pow(angle_between(node1->theta_1,node2_angle[0], &move_sign[0]),2.0) + pow(angle_between(node1->theta_2,node2_angle[1],&move_sign[1]),2.0) + pow(angle_between(node1->theta_3,node2_angle[2],&move_sign[2]),2.0) + pow(angle_between(node1->theta_4,node2_angle[3],&move_sign[3]),2.0) + pow(angle_between(node1->theta_5,node2_angle[4],&move_sign[4]),2.0);
	for (int i=0;i<5;i++){
		distance += pow(temp_angles[i],2.0);
	}

	distance = sqrt(distance);
	return distance;

}

void convert_to_unit(double* input,int size){
	double norm = 0.0;
	for (int i=0; i<size; ++i){
		norm += pow(input[i],2.0);
	}
	norm = sqrt(norm);
	for (int i=0; i<size; ++i){
		input[i] = input[i]/norm;
	}
	return;
}

double convert_to_unit_return_norm(double* input,int size){
	double norm = 0.0;
	for (int i=0; i<size; ++i){
		norm += pow(input[i],2.0);
	}
	norm = sqrt(norm);
	for (int i=0; i<size; ++i){
		input[i] = input[i]/norm;
	}
	return norm;
}

double get_neighbourhood_distance(int number_of_vertices){
	double distance;
	distance = MIN(pow(((GAMMA/VOL_HYPER)*(log(number_of_vertices)/number_of_vertices)),(float)1/(float)5),number_of_vertices);
	return distance;
}

int IsValidArmMovement(double* angles_start, double* angles_end , double* difference_angles, double norm, int* arm_sign, int numofDOFs, double* map,
	int x_size, int y_size){
	
	/*
	Trapped = -1
	Advanced = 0
	Reached = 1
	*/

	double distance = 0;
	double temporary_angles[5];
	int i,j,k;
	for (j = 0; j < numofDOFs; j++){
		temporary_angles[j] = difference_angles[j]*norm;
	}

	for (j = 0; j < numofDOFs; j++){
		if(distance < temporary_angles[j])
			distance = temporary_angles[j];
	}

	int numofsamples = (int)(distance/(PI/40));
	double temp_angles[5], old_angles[5];
	int counter = 0;


	for (i = 0; i < numofsamples; i++){
		for(j = 0; j < numofDOFs; j++){
			old_angles[j] = temp_angles[j];
			temp_angles[j] = angles_start[j] + ((double)(i+1)/(numofsamples))*(temporary_angles[j])*arm_sign[j];
			wrap_to_2pi(&temp_angles[j]);
		}
		counter ++;
		if(!IsValidArmConfiguration(temp_angles, numofDOFs, map, x_size, y_size))
		{
			if (counter <= 1){
				return -1;	
			 }
			else{
				for (k = 0; k < numofDOFs; k++){
					angles_end[k] = old_angles[k];
				}
				return 0;
			}
		}
	}  

	for (k = 0; k < numofDOFs; k++){
		angles_end[k] = temp_angles[k];
	} 
	return 1;
}


int IsValidArmMovement(double* angles_start, Node* angles_end, double* distance_neigh, int numofDOFs, double* map, int x_size, int y_size){


double difference_angles[5], temporary_angles[5];
int arm_sign[5];
double distance = 0;
int i,j,k;

double norm =  get_distance_angular(angles_end, angles_start, arm_sign, difference_angles);
*distance_neigh = norm;
convert_to_unit(difference_angles, numofDOFs);
for (j = 0; j < numofDOFs; j++){
		temporary_angles[j] = difference_angles[j]*norm;
	}

	for (j = 0; j < numofDOFs; j++){
		if(distance < temporary_angles[j])
			distance = temporary_angles[j];
	}

	int numofsamples = (int)(distance/(PI/40));
	double temp_angles[5];

	for (i = 0; i < numofsamples; i++){
		for(j = 0; j < numofDOFs; j++){
			temp_angles[j] = angles_start[j] + ((double)(i+1)/(numofsamples))*(temporary_angles[j])*-1*arm_sign[j];
			wrap_to_2pi(&temp_angles[j]);
		}
		if(!IsValidArmConfiguration(temp_angles, numofDOFs, map, x_size, y_size))
		{
			return 0;
		}
	}  

	return 1;

}

int IsValidArmMovement(Node* angles_start, Node* angles_end, double* distance_neigh, int numofDOFs, double* map, int x_size, int y_size){


double difference_angles[5], temporary_angles[5], start_angles[5];
int arm_sign[5];
double distance = 0;
int i,j,k;

start_angles[0] = angles_start->theta_1;
start_angles[1] = angles_start->theta_2;
start_angles[2] = angles_start->theta_3;
start_angles[3] = angles_start->theta_4;
start_angles[4] = angles_start->theta_5;

double norm =  get_distance_angular(angles_start, angles_end, arm_sign, difference_angles);
*distance_neigh = norm;
convert_to_unit(difference_angles, numofDOFs);
for (j = 0; j < numofDOFs; j++){
		temporary_angles[j] = difference_angles[j]*norm;
	}

	for (j = 0; j < numofDOFs; j++){
		if(distance < temporary_angles[j])
			distance = temporary_angles[j];
	}

	int numofsamples = (int)(distance/(PI/40));
	double temp_angles[5];

	for (i = 0; i < numofsamples; i++){
		for(j = 0; j < numofDOFs; j++){
			temp_angles[j] = start_angles[j] + ((double)(i+1)/(numofsamples))*(temporary_angles[j])*arm_sign[j];
			wrap_to_2pi(&temp_angles[j]);
		}
		if(!IsValidArmConfiguration(temp_angles, numofDOFs, map, x_size, y_size))
		{
			return 0;
		}
	}  

	return 1;

}
