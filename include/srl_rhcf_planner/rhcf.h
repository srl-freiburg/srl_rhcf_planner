#ifndef _RHCF_H_
#define _RHCF_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>
#include <time.h>
#include <srl_rhcf_planner/geometry/CMatrix.h>
#include <srl_rhcf_planner/geometry/realpoint.h>
#include <srl_rhcf_planner/geometry/gridUtils.h>
#include <srl_rhcf_planner/scene.h>
#include <srl_rhcf_planner/data_structures.h>

#include <srl_rhcf_planner/dynamicvoronoi/dynamicweightedvoronoi.h>

// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/LinearMath/Transform.h>





class RHCF {




public:


	RHCF();

	~RHCF();

	void readScenarioFromFile(std::string file_name);

	void readScenario(double *bot, double *goal, int Nagents, vector<Thuman> agents,
	 												double CW, double CH, double MAX_X, double MIN_X, double MAX_Y, double MIN_Y);

	void readObstacles(vector<Tobstacle> obstacles);

	void getBestPath(vector<geometry_msgs::Point> *best_path);

	void getNavGraph(vector<geometry_msgs::Point> *graph);

	void buildVoronoiDiagram();

	void buildNavigationGraph();

	void findHomotopyClasses();

	double discreteFrechet(float x1[], float y1[], float x2[], float y2[], int p, int q);

	void socialForceComputation(double** socialCostMatrix);

	double* measureDiversity(int* HCpathsCellLengths, int* HCpathsLengths, int** HCpaths);

	bool setDiscountingFactor(double d);

	double adaptingLambda(int p);


	
public:

	double social_gain_;

	int border_gain_;
	
	/// discounting factor, used in the Random Walk
	double df_; 
	/// number of homotopy class desired
	int K_;
	/// number of found homotopy classes
	int numHC_;

	// Visuals on or off?
	bool VISUALIZATION_ ;

	// timeout value in milliseconds
	int timeout_;

	// Define how fine is the border grid
	double borderInterval_;

	// save original border values (just in case)
	double XMINo_ ;
	double XMAXo_ ;
	double YMINo_ ;
	double YMAXo_ ;

	// Utils to compute quantities related to the grid
	GridUtils *grid_utils_;

	// Scene object that collects info about the scenario (robot, goal, agents, scene dimensions.. )
	Scene *scene_;

	// Voronoi Diagram Object
	DynamicVoronoi *voronoi_;

	// binary map
	bool** map_;


	// weighting map
	int** weighting_map_;

	// people directions
	std::vector<DynamicVoronoi::humanCell> human_info;
	/// internal voronoi diagram
	int** voronoiArray_;

	int** V_;

	// size in terms of number of cells
	int sizeX_;

	// size in terms of number of cells
	int sizeY_;

	// Number of vertices in the navigation graph
	int verticesCounter_;

	// Matrix with costs
	double** costMatrix_;

	// keeping edges of the paths
	float**** edgePathsCells_ ;

	// path lengths
	int** edgePathsLengths_;

	/// edges costs matrix;
	double** edgeCostMatrix_;

 	/// type of weighting to use
	int type_weighting_;

	/// gain for the weighting
	double weighting_gain_;

	/// show or not show the terminal info
	bool SHOW_LOG_INFO_;

	int robotV_ ;
	int goalV_ ;

	int *posR_;
	int *posG_;
	int *posGv_;
	int *posRV_;

	double **socialCostMatrix_;
	double 	minSF_ ;
	double  maxSF_;
	double  alpha_;

	vector< vector<RealPoint> > paths_;

	vector<Tobstacle> obstacles_;

	vector<Thuman> humans_;

	vector<geometry_msgs::Point> best_path_;

	vector<geometry_msgs::Point> nav_graph_;


	bool no_path_found_;



	double *res_diversity;
	CMatrix<float> aImageFlipped_;
	CMatrix<float> aImage_;
	CMatrix<float> aImageSF_;

};

#endif
