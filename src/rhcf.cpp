#include <srl_rhcf_planner/rhcf.h>
#define MYEPS 1e-6

using namespace std;

RHCF::RHCF(){

	grid_utils_ = new GridUtils();

	scene_ = new Scene();

	voronoi_ = new DynamicVoronoi();

	costMatrix_ = NULL;

	voronoiArray_ = NULL;

	edgePathsCells_ = NULL;

	edgePathsLengths_ = NULL;

	K_ = 2;

	VISUALIZATION_ = false;

	timeout_ = 10000;

	borderInterval_ = 1000;

	alpha_ = 0.5;

	XMINo_ = 0;

	XMAXo_ = 0;

	YMINo_ = 0;

	YMAXo_ = 0;

	map_ = NULL;

	sizeY_ = 0;

	sizeX_ = 0;

	df_ =  0.9;

	social_gain_ = 2;

	border_gain_ = 5;

	weighting_gain_ = 0.2;
 
}

RHCF::~RHCF(){

	delete grid_utils_;
	delete scene_;
	delete voronoi_;


	for(int i = 0; i<sizeX_; i++){

		delete [] map_[i];
		delete [] voronoiArray_[i];
		delete [] socialCostMatrix_[i];

	}

	delete [] map_;
	delete voronoiArray_;
	delete socialCostMatrix_;


	for(int i = 0; i<verticesCounter_; i++)
	{
		delete [] costMatrix_[i];

		for(int j = 0; j<verticesCounter_; j++)
		{

			delete [] edgePathsCells_[i][j][0];
			delete [] edgePathsCells_[i][j][1];
			delete [] edgePathsCells_[i][j];
		}
		delete [] edgePathsCells_[i];
	}
	delete [] edgePathsCells_;


	// delete edgePathsCells_;
	delete costMatrix_;
	delete edgePathsLengths_;


}



// Setting the discounting factor for the RHCF

bool RHCF::setDiscountingFactor(double d){

	df_ = d;

}

// TODO: could go in a different class
double RHCF::discreteFrechet(float x1[], float y1[], float x2[], float y2[], int p, int q)
{
	double inf = std::numeric_limits<double>::infinity();
	double dm[p+1][q+1];


	for(int i = 1; i<p+1; i++)
		dm[i][0] = inf;
	for(int i = 1; i<q+1; i++)
		dm[0][i] = inf;

	dm[0][0] = 0;

	for(int i = 1; i<p+1; i++)
  		for(int j = 1; j<q+1; j++)
  		{
    		double a = min(dm[i-1][j], min(dm[i-1][j-1], dm[i][j-1]));
    		double dist = sqrt(pow((x1[i-1] - x2[j-1]),2) + pow((y1[i-1] - y2[j-1]),2));
    		dm[i][j] = max(a,dist);
    	}

	return dm[p][q];
}


// TODO: could go in a different class
double* RHCF::measureDiversity(int* HCpathsCellLengths, int* HCpathsLengths, int** HCpaths){

	double inf = std::numeric_limits<double>::infinity();

	double diversityMatrix[numHC_][numHC_];

	for(int i = 0; i<numHC_; i++)
		for(int j = 0; j<numHC_; j++)
			diversityMatrix[i][j] = inf;

	for (int i = 0; i < numHC_; i++)
	{
		for (int j = i+1; j < numHC_; j++)
		{
			int counter1 = 0;
			int counter2 = 0;
			float x1[HCpathsCellLengths[i]];
			float x2[HCpathsCellLengths[j]];
			float y1[HCpathsCellLengths[i]];
			float y2[HCpathsCellLengths[j]];

			for(int k = 0; k < HCpathsLengths[i]-1; k++)
			{
				for(int kk = 0; kk < edgePathsLengths_[HCpaths[i][k]][HCpaths[i][k+1]]; kk++)
				{
					x1[counter1] = scene_->XMIN_ + scene_->CELLWIDTH_ * edgePathsCells_[HCpaths[i][k]][HCpaths[i][k+1]][0][kk];
					y1[counter1++] = scene_->YMIN_ + scene_->CELLHEIGHT_ * edgePathsCells_[HCpaths[i][k]][HCpaths[i][k+1]][1][kk];
				}
			}

			for(int k = 0; k < HCpathsLengths[j]-1; k++)
			{
				for(int kk = 0; kk < edgePathsLengths_[HCpaths[j][k]][HCpaths[j][k+1]]; kk++)
				{
					x2[counter2] = scene_->XMIN_ + scene_->CELLWIDTH_ * edgePathsCells_[HCpaths[j][k]][HCpaths[j][k+1]][0][kk];
					y2[counter2++] = scene_->YMIN_ + scene_->CELLHEIGHT_ * edgePathsCells_[HCpaths[j][k]][HCpaths[j][k+1]][1][kk];
				}
			}

			x1[HCpathsCellLengths[i]-1] = scene_->XMIN_ + scene_->CELLWIDTH_ * posGv_[0];
			y1[HCpathsCellLengths[i]-1] = scene_->YMIN_ + scene_->CELLHEIGHT_ * posGv_[1];
			x2[HCpathsCellLengths[j]-1] = scene_->XMIN_ + scene_->CELLWIDTH_ * posGv_[0];
			y2[HCpathsCellLengths[j]-1] = scene_->YMIN_ + scene_->CELLHEIGHT_ * posGv_[1];

			diversityMatrix[i][j] = discreteFrechet(x1, y1, x2, y2, HCpathsCellLengths[i], HCpathsCellLengths[j]);
			diversityMatrix[j][i] = diversityMatrix[i][j];

		}

	}

	double diversity = diversityMatrix[0][0];
	double robustDiversity[numHC_];
	double sum = 0;

	for(int i = 0; i<numHC_; i++)
	{
		robustDiversity[i] = diversityMatrix[i][0];
		for(int j = 0; j<numHC_; j++)
		{
			if(diversityMatrix[i][j] < diversity) diversity = diversityMatrix[i][j];
			if(diversityMatrix[i][j] < robustDiversity[i]) robustDiversity[i] = diversityMatrix[i][j];
		}
		sum+=robustDiversity[i];
	}
	sum/=numHC_;
	res_diversity = new double[2];
	res_diversity[0] = diversity;
	res_diversity[1] = sum;

	return res_diversity;
}



/// Compute the lambda according to the velocity of the human with index p
/// if it is still lambda is higher, close to one
double  RHCF::adaptingLambda(int p){

	double h_v = sqrt(humans_[p].vx*humans_[p].vx + humans_[p].vy*humans_[p].vy);
	double Kg = 6;
	double labdamax = 0.1;
				
	return (1 - (1-labdamax)*tanh(Kg*h_v) );


}

// TODO: could be inserted in a different class
void RHCF::socialForceComputation(double** socialCostMatrix){

	double inf = std::numeric_limits<double>::infinity();

	//-----> Social costs computation
	minSF_ = inf;
	maxSF_ = 0;
	int peopleIJ[scene_->N_agents_][2];		// i and j grid coordinates for all people
	double peopleDir[scene_->N_agents_][2];	// inverted vectors of direction

	human_info.clear();

	for(int p = 0; p<scene_->N_agents_; p++)
	{

		peopleIJ[p][0] = grid_utils_->xyToI(scene_->xpeople_[p][0], scene_->xpeople_[p][1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
		peopleIJ[p][1] = grid_utils_->xyToJ(scene_->xpeople_[p][0], scene_->xpeople_[p][1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);


		peopleDir[p][0] = (-1)*cos(scene_->xpeople_[p][2]);
		peopleDir[p][1] = (-1)*sin(scene_->xpeople_[p][2]);

		DynamicVoronoi::humanCell hi;
		hi.x = peopleIJ[p][0];
		hi.y = peopleIJ[p][1];
		hi.vx = - peopleDir[p][0];
		hi.vy = - peopleDir[p][1];
		human_info.push_back(hi);
		ROS_INFO_STREAM("p = " << p << ", peopleDir1 = " << peopleDir[p][0] << ", peopleDir2 = " << peopleDir[p][1]);

	}

	double ak = 2;          // magnitude of social force to pedestrians
	double bk = 1;           // range of social force to pedestrians
	double lambda = 0.1;     // anisotropic influence factor

	double rij;        // sum of agent radii
	double dij;        // fix distance between agent i and j

	double nij[2];    // normalized vector pointing from agent i to j: this is the free variable

	alpha_ = 0.5; // weight of length penalazier, cost(edge) = alpha*social + (1-alpha)*length

	for(int i = 0; i<sizeX_; i++)
		for(int j = 0; j<sizeY_; j++)
		{
			double socialForce = 0;
			for(int p = 0; p<scene_->N_agents_; p++)
			{
				rij = 0.4;
				dij = sqrt(scene_->CELLWIDTH_*scene_->CELLWIDTH_*(peopleIJ[p][0]-i)*(peopleIJ[p][0]-i) +
						scene_->CELLHEIGHT_*scene_->CELLHEIGHT_*(peopleIJ[p][1]-j)*(peopleIJ[p][1]-j));

				int xi = i - peopleIJ[p][0];
				int yi = j - peopleIJ[p][1];

				double x = (double)xi;
				double y = (double)yi;

				double l = sqrt(x*x + y*y );
				
				lambda = adaptingLambda(p);
				
				// ROS_INFO("Lambda %f, h_v %f, vx %f, vy %F", lambda, h_v, humans_[p].vx, humans_[p].vy);
				if(l<MYEPS){


				}else{

					x = x/(l);
					y = y/(l);

					double f1 = ( pow((ak*exp((rij-dij)/bk) * x * (lambda + ((1-lambda)*0.5*(1 - x*peopleDir[p][0] - y*peopleDir[p][1])))),2));
					double f2 = ( pow((ak*exp((rij-dij)/bk) * y * (lambda + ((1-lambda)*0.5*(1 - x*peopleDir[p][0] - y*peopleDir[p][1])))),2));
					double f = f1 + f2;

					double fani = sqrt((f));

					socialForce+=fani;

				}


			}


			socialCostMatrix[i][j] = socialForce;
			if(socialCostMatrix[i][j]<minSF_) minSF_ = socialCostMatrix[i][j];
			if(socialCostMatrix[i][j]>maxSF_) maxSF_ = socialCostMatrix[i][j];
		}



}

void RHCF::getNavGraph(vector<geometry_msgs::Point> *graph){

	ROS_INFO("Gettin graph points");
	int n_points = (int)nav_graph_.size();

	for(int i = 0; i<n_points	; i++){

			geometry_msgs::Point p ;
			p.x = nav_graph_[i].x;
			p.y = nav_graph_[i].y;
			p.z = nav_graph_[i].z;

			graph->push_back(p);

	}


}



void RHCF::getBestPath(vector<geometry_msgs::Point> *best_path){

	ROS_INFO("Gettin best geometric path");
	int n_points = (int)best_path_.size();

	for(int i = 0; i<n_points	; i++){

			geometry_msgs::Point p ;
			p.x = best_path_[i].x;
			p.y = best_path_[i].y;
			p.z = best_path_[i].z;

      ROS_INFO("Best Path point %d, %f %f", i, p.x, p.y);
			best_path->push_back(p);

	}


}

void RHCF::readObstacles(vector<Tobstacle> obsts){

	ROS_INFO("Setting Obstacles in the RHCF planner");
	int n_obsts = (int)obsts.size();

	for(int i =0; i<n_obsts; i++ ){

		Tobstacle o;
		o.x = obsts[i].x;
		o.y = obsts[i].y;
		o.z = obsts[i].z;
		o.cell_width = obsts[i].cell_width;
		o.cell_height = obsts[i].cell_height;
		obstacles_.push_back(o);

	}

}

void RHCF::readScenario(double *bot, double *goal, int Nagents, vector<Thuman> agents,
 												double CW, double CH, double MAX_X, double MIN_X, double MAX_Y, double MIN_Y){

	double xrobot[3];
	double xgoal[2];
	double** xpeople = NULL;
	double CELLWIDTH = 0;
	double CELLHEIGHT = 0;
	double XMIN = 0;
	double XMAX = 0;
	double YMIN = 0;
	double YMAX = 0;
	// Number of people
	int N = 0;

	// setting grid propertiers
	CELLWIDTH = CW;
	CELLHEIGHT = CH;
	XMIN = MIN_X;
	XMAX = MAX_X;
	YMIN = MIN_Y;
	YMAX = MAX_Y;

	/// set robot pose
	for(int i=0; i<3; i++){

		xrobot[0] = bot[0];
		xrobot[1] = bot[1];
		xrobot[2] = bot[2];
	}

	/// set goal position
	for(int i=0; i<2; i++){

		xgoal[0] = goal[0];
		xgoal[1] = goal[1];
	}

	/// set agents
	N = (int) agents.size();
	ROS_INFO("Agents in the scene %d", N);
	scene_->setAgents(N);

	/// for each agent set the proper coordinate
	humans_.clear();
	for(int i = 0; i< N; i++){

		Thuman h = agents[i];
		humans_.push_back(h);
		scene_->setAgentCoordinate(i, 0, h.x);
		scene_->setAgentCoordinate(i, 1, h.y);
		scene_->setAgentCoordinate(i, 2, h.z);

	}

	// Visuals on or off?
	VISUALIZATION_ = false;

	// timeout value in milliseconds
	timeout_ = 50;

	// Define how fine is the border grid
	borderInterval_ = (XMAX-XMIN);

	// save original border values (just in case)
	XMINo_ = XMIN;
	XMAXo_ = XMAX;
	YMINo_ = YMIN;
	YMAXo_ = YMAX;

	// border extension
	// (this is required to ensure correct building
	// of the voronoi-based navigation graph, that is, connected)
	double boxW = (XMAX-XMIN)/4;
	double boxH = (YMAX-YMIN)/4;

	// new border values

	//XMIN = XMIN - boxW;
	//XMAX = XMAX + boxW;
	//YMIN = YMIN - boxH;
	//YMAX = YMAX + boxH;

	scene_->setRobot( xrobot[0], xrobot[1], xrobot[2] );
	scene_->setGoal( xgoal[0],  xgoal[1]);
	scene_->setScene(CELLWIDTH, CELLHEIGHT, XMIN, XMAX, YMIN, YMAX);



	// Pixel size of the grid
	sizeX_ = (scene_->XMAX_ - scene_->XMIN_)/scene_->CELLWIDTH_;
	sizeY_ = (scene_->YMAX_ - scene_->YMIN_)/scene_->CELLHEIGHT_;


 	ROS_INFO("sizeX_ %d sizeY_ %d ", sizeX_, sizeY_);

	voronoiArray_ = new int*[sizeX_];
		for(int i = 0; i < sizeX_; ++i)
			voronoiArray_[i] = new int[sizeY_];

	for (int i=0; i<sizeX_; i++)
		for (int j=0; j<sizeY_; j++)
			voronoiArray_[i][j] = 0;

	socialCostMatrix_ = new double*[sizeX_];

	for(int i = 0; i < sizeX_; ++i)
			socialCostMatrix_[i] = new double[sizeY_];

	for (int i=0; i<sizeX_; i++)
		for (int j=0; j<sizeY_; j++)
			socialCostMatrix_[i][j] = 0;

}


void RHCF::readScenarioFromFile(std::string file_name){


	double xrobot[3];
	double xgoal[2];
	double** xpeople = NULL;
	double CELLWIDTH = 0;
	double CELLHEIGHT = 0;
	double XMIN = 0;
	double XMAX = 0;
	double YMIN = 0;
	double YMAX = 0;
	// Number of people
	int N = 0;
	ifstream myfile(file_name.c_str());
	//ifstream myfile ("input.txt");
	//ifstream myfile ("two_people.txt");
	// ifstream myfile ("people_wall.txt");
	//ifstream myfile ("sprunk.txt");
	//ifstream myfile ("one_person.txt");
	//ifstream myfile ("crowd_anim.txt");
	if (myfile.is_open())
	{
		int people_counter = 0;
		bool get_people = false;
		string line;
		string prev_line = "";

    	while ( getline (myfile,line) )
    	{
      		//cout << line << '\n';
      		int counter = 0;

      		istringstream iss(line);
    		while (iss)
    		{
        		string sub;
        		iss >> sub;
        		if(sub.length() == 0) continue;
        		//cout << "Substring: " << sub << endl;

        		if(prev_line == "xrobot" && counter <= 2)
        		{
        			xrobot[counter] = std::stod(sub);
        			counter+=1;
        		}
        		else if(prev_line == "xgoal" && counter <= 1)
        		{
        			xgoal[counter] = stod(sub);
        			counter+=1;
        		}
        		else if(prev_line == "XMIN") XMIN = stod(sub);
        		else if(prev_line == "XMAX") XMAX = stod(sub);
        		else if(prev_line == "YMAX") YMAX = stod(sub);
        		else if(prev_line == "YMIN") YMIN = stod(sub);
        		else if(prev_line == "CELLWIDTH") CELLWIDTH = stod(sub);
        		else if(prev_line == "CELLHEIGHT") CELLHEIGHT = stod(sub);
        		else if(get_people == true && line == "CELLWIDTH") get_people = false;
        		else if(sub == "xpeople") get_people = true;

        		if(get_people == true && prev_line == "xpeople" && counter == 0)
        		{
        			N = stoi(sub);
					scene_->setAgents(N);
        			counter++;

        		}
        		else if(get_people == true && people_counter < N && counter <= 2)
        		{
							scene_->setAgentCoordinate(people_counter, counter, stod(sub));

        			if (counter == 2) people_counter++;
        			counter++;
        		}
    		}
    		counter = 0;
    		prev_line = line;
    	}
    	myfile.close();
	}else{

		ROS_WARN("File not properly read!!! %s", file_name.c_str());
		char cwd[1024];
   		if (getcwd(cwd, sizeof(cwd)) != NULL)
       		fprintf(stdout, "Current working dir: %s\n", cwd);
   		else
       		perror("getcwd() error");
		exit(0);
	}



    // Visuals on or off?
	VISUALIZATION_ = false;

	// timeout value in milliseconds
	timeout_ = 10000;

	// Define how fine is the border grid
	borderInterval_ = (XMAX-XMIN);

	// save original border values (just in case)
	XMINo_ = XMIN;
	XMAXo_ = XMAX;
	YMINo_ = YMIN;
	YMAXo_ = YMAX;

	// border extension
	// (this is required to ensure correct building
	// of the voronoi-based navigation graph, that is, connected)
	double boxW = (XMAX-XMIN)/4;
	double boxH = (YMAX-YMIN)/4;

	// new border values

	XMIN = XMIN - boxW;
	XMAX = XMAX + boxW;
	YMIN = YMIN - boxH;
	YMAX = YMAX + boxH;

	scene_->setRobot( xrobot[0], xrobot[1], xrobot[2] );
	scene_->setGoal( xgoal[0],  xgoal[1]);
	scene_->setScene(CELLWIDTH, CELLHEIGHT, XMIN, XMAX, YMIN, YMAX);



	// Pixel size of the grid
	sizeX_ = (scene_->XMAX_ - scene_->XMIN_)/scene_->CELLWIDTH_;
	sizeY_ = (scene_->YMAX_ - scene_->YMIN_)/scene_->CELLHEIGHT_;



	voronoiArray_ = new int*[sizeX_];
    for(int i = 0; i < sizeX_; ++i)
       voronoiArray_[i] = new int[sizeY_];


   	socialCostMatrix_ = new double*[sizeX_];

	for(int i = 0; i < sizeX_; ++i)
	    socialCostMatrix_[i] = new double[sizeY_];
}



void RHCF::buildVoronoiDiagram(){

	//========================------------------- VORONOI -----------------------========================


	weighting_map_ = new int*[sizeX_];

	// Initialize the pixel grid [sizeX x sizeY] with occupied space
	for(int i = 0; i<sizeX_; i++)
	{
		weighting_map_[i] = new int[sizeY_];
		
		for(int j = 0; j<sizeY_; j++)
			weighting_map_[i][j] = 0;
	}



	map_ = new bool*[sizeX_];

	// Initialize the pixel grid [sizeX x sizeY] with occupied space
	for(int i = 0; i<sizeX_; i++)
	{
		map_[i] = new bool[sizeY_];
		

		for(int j = 0; j<sizeY_; j++)
			map_[i][j] = true;
	}

	// Free the occupied space except for 1 pixel thin border
	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
			map_[i][j] = false;



	/// Setting occupied cells
	int n_obsts = (int) obstacles_.size();
	for(int i =0; i<n_obsts; i++ ){

		int persI = grid_utils_->xyToI(obstacles_[i].x, obstacles_[i].y, sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		int persJ = grid_utils_->xyToJ(obstacles_[i].x, obstacles_[i].y, sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		map_[persI][persJ] = true;
	}

	// Add N people to the map_, each person occupies one pixel
	ROS_INFO("Adding %d humans to map_", scene_->N_agents_);
	for(int i = 0; i<scene_->N_agents_; i++)
	{
		int persI = grid_utils_->xyToI(scene_->xpeople_[i][0], scene_->xpeople_[i][1], sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		int persJ = grid_utils_->xyToJ(scene_->xpeople_[i][0], scene_->xpeople_[i][1], sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		map_[persI][persJ] = true;
   }


    CMatrix<float> am(sizeX_, sizeY_ ,255);
    aImage_ = am;


    CMatrix<float> aSF(sizeX_,sizeY_,255);
    aImageSF_ = aSF;

    /// to remove misleading behaviours when having only false positive in the scene
	//int r_i, r_j, g_i, g_j ;

	// r_i = grid_utils_->xyToI(scene_->xrobot_[0], scene_->xrobot_[1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	// r_j = grid_utils_->xyToJ(scene_->xrobot_[0], scene_->xrobot_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	// map_[r_i][r_j] = true;


	// g_i = grid_utils_->xyToI(scene_->xgoal_[0], scene_->xgoal_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	// g_j = grid_utils_->xyToJ(scene_->xgoal_[0], scene_->xgoal_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	// map_[g_i][g_j] = true;



		//-----> Social costs computation
		// Start filling the result image
	if(scene_->N_agents_>0){

		ROS_WARN("Social Force Computation!! ");
        socialForceComputation(socialCostMatrix_);

        //cout << "Social Force Matrix:" << endl;
        for(int i = 0; i<sizeX_; i++)
        {
          for(int j = 0; j<sizeY_; j++)
          {
            aImageSF_(i,j) = 255 - (255*socialCostMatrix_[i][j])/maxSF_;
            aImage_(i,j) = 255 - (255*socialCostMatrix_[i][j])/maxSF_;

            int a = 255 - (255*socialCostMatrix_[i][j])/maxSF_;
            

            if(a<0 || a>255){
              ROS_ERROR("Value a not in the range %d", a);
              ROS_ERROR("Value a not in the range %f", socialCostMatrix_[i][j]);
              ROS_ERROR("Value a not in the range %f", maxSF_);
              a = 0;
            }


            weighting_map_[i][j] = a;

            
			
        }
        //cout << socialCostMatrix[i][j] << " ";
        //cout << endl;
        }

       	if(SHOW_LOG_INFO_)
	        aImageSF_.writeToPGM("images/SF.pgm");
		//<-----
	}

			/// Copying obstacles into the image
	for(int i =0; i<n_obsts; i++ ){

		int persI = grid_utils_->xyToI(obstacles_[i].x, obstacles_[i].y, sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		int persJ = grid_utils_->xyToJ(obstacles_[i].x, obstacles_[i].y, sizeX_, scene_->CELLWIDTH_,
			scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

		aImage_(persI, persJ) = 0;
		aImageSF_(persI, persJ) = 0;
	}





	// Initialize the Voronoi
	voronoi_->initializeMap(sizeX_, sizeY_, map_);
	

	voronoi_->initializeWeightingMap(sizeX_, sizeY_, weighting_map_);

	voronoi_->initializeHumanInfo(sizeX_, sizeY_, human_info);


	ROS_INFO("Type weighting %d", type_weighting_);
	
	if(scene_->N_agents_>0){

		voronoi_->setTypeWeighting(type_weighting_);
	}
	else{
		/// no agents use euclidean distance
		voronoi_->setTypeWeighting(0);
	}

    voronoi_->settingGains(social_gain_, border_gain_, weighting_gain_);

	voronoi_->update();
	
	// voronoi_->prune();

	/// If we have agents in the scen
	if(scene_->N_agents_>0){
		
		voronoi_->updateAlternativePrunedDiagramWithHumans();

	}else{
		
		voronoi_->updateAlternativePrunedDiagram();

	}

	voronoi_->visualize("images/voronoiDiagram.ppm"); /// TODO: check name!!!



	// Check for "squares" - quit in unlucky case
	for(int i = 0; i<sizeX_-1; i++)
		for(int j = 0; j<sizeY_-1; j++)
			if(voronoi_->isVoronoiAlternative(i,j) && voronoi_->isVoronoiAlternative(i+1,j) &&
				voronoi_->isVoronoiAlternative(i,j+1) && voronoi_->isVoronoiAlternative(i+1,j+1))
			{
				ROS_WARN_STREAM("\n[WARNING] Square found.\n\n");
				//return 1;
			}

	// Number of vertices in the navigation graph
	verticesCounter_ = 0;

	// TODO: DELETE THIS
	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
			{	

				voronoiArray_[i][j] = 0;


				if(voronoiArray_[i][j] == 2)
					ROS_WARN_STREAM("\n[WARNING] Some 2s are in the array beforehand!!!\n\n");
			}


	// Round one - create all main vertices (pixels with power = 3)
	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
			if(voronoi_->isVoronoiAlternative(i,j) && (voronoi_->getNumVoronoiNeighborsAlternative(i,j) >= 3) &&
				(voronoiArray_[i-1][j] != 2) && (voronoiArray_[i][j-1] != 2))
			{
				voronoiArray_[i][j] = 2;
				verticesCounter_++;
				ROS_WARN("\n[ADDING 2, place1] %d %d \n",i,j);
			}

	bool noVertices = (verticesCounter_ == 0);

	// Round two - create all additional vertices on edges



	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
		{

			if(voronoi_->getNumVoronoiNeighborsAlternative(i,j) < 2) continue;

			/// Initializing first available pixel as vertex
			// if(noVertices && voronoi_->isVoronoiAlternative(i,j))
			// {
			// 	voronoiArray_[i][j] = 2;
			// 	verticesCounter_++;
			// 	ROS_WARN("\n[ADDING 2, place2 2nd round 1] %d  %d \n",i,j);
			// 	noVertices = false;
			// }

			if(voronoi_->isVoronoiAlternative(i,j) && (voronoi_->getNumVoronoiNeighborsAlternative(i,j) >= 3)
				&& voronoiArray_[i][j] == 2)
			{
				// List of possible distance=2 neighbors of a vertice (i,j)
				int nx[8] = {i-2, i-1, i, i+1, i+2, i+1, i, i-1};
				int ny[8] = {j, j-1, j-2, j-1, j, j+1, j+2, j+1};

				// For all the possible distance=2 neighbors...
				for(int k = 0; k<8; k++)
				{
					// ... if it belongs to the voronoi diagram and is not a main vertice...
					if(voronoi_->isVoronoiAlternative(nx[k],ny[k]) &&
						(voronoi_->getNumVoronoiNeighborsAlternative(nx[k],ny[k]) == 2))
					{
						// ... and it does not have a neighbour which is a main vertice...
						if((voronoiArray_[nx[k]-1][ny[k]] != 2) && (voronoiArray_[nx[k]+1][ny[k]] != 2) &&
							(voronoiArray_[nx[k]][ny[k]-1] != 2) && (voronoiArray_[nx[k]][ny[k]+1] != 2))
						{
							// ... and it's not marked as a vertice yet.
							if(voronoiArray_[nx[k]][ny[k]] != 2)
							{
								voronoiArray_[nx[k]][ny[k]] = 2;
								verticesCounter_++;
								ROS_WARN("\n[ADDING 2, place3 2nd round 2] %d  %d \n",nx[k],ny[k]);
							}
						}
					}
				}
			}
			else if(voronoi_->isVoronoiAlternative(i,j) && (voronoi_->getNumVoronoiNeighborsAlternative(i,j) >= 3))
					voronoiArray_[i][j] = 1;
			else if(voronoi_->isVoronoiAlternative(i,j) && voronoiArray_[i][j] != 2) voronoiArray_[i][j] = 1;
			else if(voronoiArray_[i][j] != 2) voronoiArray_[i][j] = 0;
		}




	// Distance to robot
	float distR = sqrt(sizeX_*sizeX_ + sizeY_*sizeY_);

	// Robot entrance into the voronoi diagram - the closest voronoi pixel to the initial position

	posR_ = new int [2];
	posR_[0] = -1; posR_[1] = -1;

	posG_= new int [2];
	posG_[0] = -1; posG_[1] = -1;

	posGv_= new int [2];
	posGv_[0] = -1; posGv_[1] = -1;

	posRV_= new int [2];
	posRV_[0] = -1; posRV_[1] = -1;

	// Distance to goal
	float distG = sqrt(sizeX_*sizeX_ + sizeY_*sizeY_);

	//int xyToI(double x, double y, int xSize, double xStep, double yStep, double xMin, double yMin)
	posR_[0] = grid_utils_->xyToI(scene_->xrobot_[0], scene_->xrobot_[1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	posR_[1] = grid_utils_->xyToJ(scene_->xrobot_[0], scene_->xrobot_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	posG_[0] = grid_utils_->xyToI(scene_->xgoal_[0], scene_->xgoal_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	posG_[1] = grid_utils_->xyToJ(scene_->xgoal_[0], scene_->xgoal_[1],  sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);


	// Find robot entrance into the voronoi diagram - closest voronoi pixel
	float robotOffset = 0;
	float robotOffset_final = 0;
	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
			// Robot enters only if the voronoi pixel has no neighbor vertices
			if(voronoiArray_[i][j] == 1 && voronoiArray_[i+1][j] != 2 && voronoiArray_[i-1][j] != 2 &&
				voronoiArray_[i][j-1] != 2 && voronoiArray_[i][j+1] != 2)
			{
				robotOffset = sqrt((posR_[0]-i)*(posR_[0]-i) + (posR_[1]-j)*(posR_[1]-j));
				if(robotOffset<distR)
				{
					distR = robotOffset;
					robotOffset_final = robotOffset;
					posRV_[0] = i;
					posRV_[1] = j;

						if(SHOW_LOG_INFO_)
						{
							ROS_INFO_STREAM("Robot is now in: " << posRV_[0] << ", " << posRV_[1] << ", distR = " << distR  <<  ", robotOffset = " << robotOffset_final) ;
						}

				}
			}

	// Find goal entrance into the voronoi diagram - closest voronoi pixel
	// Need separate cycles for R and G, so that they don't end up in neighboring vertices
	if (voronoiArray_[posRV_[0]][posRV_[1]] < 2)
	{
		voronoiArray_[posRV_[0]][posRV_[1]] = 2;
		verticesCounter_++;
		ROS_WARN("\n[ADDING 2, place4 Robot] %d  %d \n",posRV_[0],posRV_[1]);
		ROS_INFO("Robot added, counter incremented");
	}

	if(noVertices)
	{	
		if (voronoiArray_[posRV_[0]-2][posRV_[1]] == 1)
		{
			voronoiArray_[posRV_[0]-2][posRV_[1]] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]-2,posRV_[1]);
			ROS_INFO("Fake V added, counter incremented");
		} else 	if (voronoiArray_[posRV_[0]-1][posRV_[1]-1] == 1)
		{
			voronoiArray_[posRV_[0]-1][posRV_[1]-1] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]-1,posRV_[1]-1);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]][posRV_[1]-2] == 1)
		{
			voronoiArray_[posRV_[0]][posRV_[1]-2] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0],posRV_[1]-2);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]+1][posRV_[1]-1] == 1)
		{
			voronoiArray_[posRV_[0]+1][posRV_[1]-1] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]+1,posRV_[1]-1);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]+2][posRV_[1]] == 1)
		{
			voronoiArray_[posRV_[0]+2][posRV_[1]] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]+2,posRV_[1]);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]+1][posRV_[1]+1] == 1)
		{
			voronoiArray_[posRV_[0]+1][posRV_[1]+1] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]+1,posRV_[1]+1);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]][posRV_[1]+2] == 1)
		{
			voronoiArray_[posRV_[0]][posRV_[1]+2] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0],posRV_[1]+2);
			ROS_INFO("Fake V added, counter incremented");
			
		} else if (voronoiArray_[posRV_[0]-1][posRV_[1]+1] == 1)
		{
			voronoiArray_[posRV_[0]-1][posRV_[1]+1] = 2;
			verticesCounter_++;
			ROS_WARN("\n[ADDING 2, in noVertices cycle after robot is added] %d  %d \n",posRV_[0]-1,posRV_[1]+1);
			ROS_INFO("Fake V added, counter incremented");
			
		} else ROS_WARN("NO FAKE V ADDED, MULTIPLE EDGES WILL BE FOUND!!!");
	}

	// Find robot and goal entrances into the voronoi diagram - closest voronoi pixels respectively
	float goalOffset = 0;
	float goalOffset_final = 0;
	for(int i = 1; i<sizeX_-1; i++)
		for(int j = 1; j<sizeY_-1; j++)
			// Robot enters only if the voronoi pixel is not yet a vertice and has no neighbor vertices
			if(voronoiArray_[i][j] == 1 && voronoiArray_[i+1][j] != 2 && voronoiArray_[i-1][j] != 2 &&
				voronoiArray_[i][j-1] != 2 && voronoiArray_[i][j+1] != 2)
			{
				goalOffset = sqrt((posG_[0]-i)*(posG_[0]-i) + (posG_[1]-j)*(posG_[1]-j));
				if(goalOffset<distG)
				{
					distG = goalOffset;
					goalOffset_final = goalOffset ;
					posGv_[0] = i;
					posGv_[1] = j;

					if(SHOW_LOG_INFO_)
					{
						ROS_INFO_STREAM("Goal is now in: " << posGv_[0] << ", " << posGv_[1] << ", distG = " << distG << ", goalOffset = "<< goalOffset);
					}

				}
			}


	if( posGv_[0]==-1 && posGv_[1]==-1){

		posGv_[0] = posRV_[0];
		posGv_[1] = posRV_[1];
		ROS_WARN("Mapping goal into robot voronoi cell");
	} else
	{
		voronoiArray_[posGv_[0]][posGv_[1]] = 2;
		verticesCounter_++;
		ROS_WARN("\n[ADDING 2, place5 Goal] %d  %d \n", posGv_[0], posGv_[1] );
	}

	if(SHOW_LOG_INFO_)
	{
		ROS_INFO_STREAM("Robot is now in: " << posRV_[0] << ", " << posRV_[1] << ", distR = " << distR  <<  ", robotOffset = " << robotOffset_final) ;
		ROS_INFO_STREAM("Goal is now in: " << posGv_[0] << ", " << posGv_[1] << ", distG = " << distG << ", goalOffset = "<< goalOffset_final);
		ROS_INFO_STREAM("verticesCounter_ = " << verticesCounter_);
	}
	// Now we have all vertices prepared in the binary array
	// Collect there coordinates in V matrix
	V_ = new int*[verticesCounter_];
    for(int i = 0; i < verticesCounter_; ++i)
       V_[i] = new int[2];


	// int V[verticesCounter_][2];
	int counter = 0;

	// Add original XMIN-XMAX visuals
	int XminYminStartI = grid_utils_->xyToI(XMINo_, YMINo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XminYminStartJ = grid_utils_->xyToJ(XMINo_, YMINo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XmaxYminStartI = grid_utils_->xyToI(XMAXo_, YMINo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XmaxYminStartJ = grid_utils_->xyToJ(XMAXo_, YMINo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XminYmaxStartI = grid_utils_->xyToI(XMINo_, YMAXo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XminYmaxStartJ = grid_utils_->xyToJ(XMINo_, YMAXo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XmaxYmaxStartI = grid_utils_->xyToI(XMAXo_, YMAXo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
	int XmaxYmaxStartJ = grid_utils_->xyToJ(XMAXo_, YMAXo_, sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);

	// aImage_.drawLine(XminYminStartI,XminYminStartJ,XminYmaxStartI,XminYmaxStartJ,247);
	// aImage_.drawLine(XminYminStartI,XminYminStartJ,XmaxYminStartI,XmaxYminStartJ,247);
	// aImage_.drawLine(XmaxYmaxStartI,XmaxYmaxStartJ,XminYmaxStartI,XminYmaxStartJ,247);
	// aImage_.drawLine(XmaxYmaxStartI,XmaxYmaxStartJ,XmaxYminStartI,XmaxYminStartJ,247);

	// Collect all vertices in V matrix
	for(int i = 1; i<sizeX_-1; i++){
		// if(counter>=verticesCounter_){
		// 	ROS_WARN("counter>=verticesCounter_");
		// 	break;
		// }
		for(int j = 1; j<sizeY_-1; j++){
			if(voronoiArray_[i][j] == 2)
			{
				V_[counter][0] = i;
				V_[counter][1] = j;

				if(SHOW_LOG_INFO_)
					ROS_INFO_STREAM("Counter = " <<  counter << " for i = " << i << ", j = " << j);

				counter++;

				// if(counter>=verticesCounter_){
				// 	ROS_WARN("counter>=verticesCounter_");
				// 	break;
				// }

				(aImage_)(i,j) = 0;



			}
			else if(voronoiArray_[i][j] == 1){
				// TODO: check if properly copied
				(aImage_)(i,j) = 200;
				geometry_msgs::Point p;
				p.x = i* scene_->CELLWIDTH_ +  scene_->XMIN_;
				p.y = j* scene_->CELLHEIGHT_ +  scene_->YMIN_;
				p.z = 0;
				nav_graph_.push_back(p);


			}

		}
	}

	// Draw robot and goal onto the image
// Draw robot and goal onto the image
	aImage_(posR_[0]-1,posR_[1]-1) = 100;
	aImage_(posR_[0]-2,posR_[1]-2) = 100;
	aImage_(posR_[0]-1,posR_[1]+1) = 100;
	aImage_(posR_[0]-2,posR_[1]+2) = 100;
	aImage_(posR_[0]+1,posR_[1]-1) = 100;
	aImage_(posR_[0]+2,posR_[1]-2) = 100;
	aImage_(posR_[0]+1,posR_[1]+1) = 100;
	aImage_(posR_[0]+2,posR_[1]+2) = 100;
	aImage_(posR_[0],posR_[1]) = 100;

	aImage_(posG_[0]-2,posG_[1]) = 220;
	aImage_(posG_[0]-2,posG_[1]-1) = 220;
	aImage_(posG_[0]-2,posG_[1]-2) = 220;
	aImage_(posG_[0]-1,posG_[1]-2) = 220;
	aImage_(posG_[0],posG_[1]-2) = 220;
	aImage_(posG_[0]+1,posG_[1]-2) = 220;
	aImage_(posG_[0]+2,posG_[1]-2) = 220;
	aImage_(posG_[0]+2,posG_[1]-1) = 220;
	aImage_(posG_[0]+2,posG_[1]) = 220;
	aImage_(posG_[0]+2,posG_[1]+1) = 220;
	aImage_(posG_[0]+2,posG_[1]+2) = 220;
	aImage_(posG_[0]+1,posG_[1]+2) = 220;
	aImage_(posG_[0],posG_[1]+2) = 220;
	aImage_(posG_[0]-1,posG_[1]+2) = 220;
	aImage_(posG_[0]-2,posG_[1]+2) = 220;
	aImage_(posG_[0]-2,posG_[1]+1) = 220;
	aImage_(posG_[0],posG_[1]) = 220;

	//aImage_(posG[0]-1,posG[1]) = 220;
	//aImage_(posG[0]+1,posG[1]) = 220;
	//aImage_(posG[0],posG[1]-1) = 220;
	//aImage_(posG[0],posG[1]+1) = 220;

	aImage_(posRV_[0],posRV_[1]) = 100;
	aImage_(posGv_[0],posGv_[1]) = 220;

// Draw robot and goal onto the image
	aImageSF_(posR_[0]-1,posR_[1]-1) = 100;
	aImageSF_(posR_[0]-2,posR_[1]-2) = 100;
	aImageSF_(posR_[0]-1,posR_[1]+1) = 100;
	aImageSF_(posR_[0]-2,posR_[1]+2) = 100;
	aImageSF_(posR_[0]+1,posR_[1]-1) = 100;
	aImageSF_(posR_[0]+2,posR_[1]-2) = 100;
	aImageSF_(posR_[0]+1,posR_[1]+1) = 100;
	aImageSF_(posR_[0]+2,posR_[1]+2) = 100;
	aImageSF_(posR_[0],posR_[1]) = 100;

	aImageSF_(posG_[0]-2,posG_[1]) = 220;
	aImageSF_(posG_[0]-2,posG_[1]-1) = 220;
	aImageSF_(posG_[0]-2,posG_[1]-2) = 220;
	aImageSF_(posG_[0]-1,posG_[1]-2) = 220;
	aImageSF_(posG_[0],posG_[1]-2) = 220;
	aImageSF_(posG_[0]+1,posG_[1]-2) = 220;
	aImageSF_(posG_[0]+2,posG_[1]-2) = 220;
	aImageSF_(posG_[0]+2,posG_[1]-1) = 220;
	aImageSF_(posG_[0]+2,posG_[1]) = 220;
	aImageSF_(posG_[0]+2,posG_[1]+1) = 220;
	aImageSF_(posG_[0]+2,posG_[1]+2) = 220;
	aImageSF_(posG_[0]+1,posG_[1]+2) = 220;
	aImageSF_(posG_[0],posG_[1]+2) = 220;
	aImageSF_(posG_[0]-1,posG_[1]+2) = 220;
	aImageSF_(posG_[0]-2,posG_[1]+2) = 220;
	aImageSF_(posG_[0]-2,posG_[1]+1) = 220;
	aImageSF_(posG_[0],posG_[1]) = 220;


	aImageSF_(posRV_[0],posRV_[1]) = 100;
	aImageSF_(posGv_[0],posGv_[1]) = 220;
	// TODO: give appropriate names!
    if(SHOW_LOG_INFO_)
	 	aImageSF_.writeToPGM("images/desc1.pgm");

	if(SHOW_LOG_INFO_)
		aImage_.writeToPGM("images/desc2.pgm");



	if(SHOW_LOG_INFO_)
		ROS_INFO("Draw people onto the image %d", scene_->N_agents_);

	// Draw people onto the image
	for(int i = 0; i<scene_->N_agents_; i++)
	{
		int persI = grid_utils_->xyToI( scene_->xpeople_[i][0], scene_->xpeople_[i][1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
		int persJ = grid_utils_->xyToJ( scene_->xpeople_[i][0], scene_->xpeople_[i][1], sizeX_, scene_->CELLWIDTH_, scene_->CELLHEIGHT_, scene_->XMIN_, scene_->YMIN_);
		(aImage_)(persI,persJ) = 0;
    }


	// Find ID's of robot and goal entrance vertices in the V matrix
	robotV_ = -1;
	goalV_ = -1;

	if(SHOW_LOG_INFO_)
    ROS_INFO("verticesCounter_ %d", verticesCounter_);

	for(int i = 0; i<verticesCounter_; i++)
	{
		if(V_[i][0] == posRV_[0] && V_[i][1] == posRV_[1] && robotV_ < 0)
		{
			int temp[2] = {V_[verticesCounter_-2][0],V_[verticesCounter_-2][1]};
			V_[verticesCounter_-2][0] = V_[i][0];
			V_[verticesCounter_-2][1] = V_[i][1];
			V_[i][0] = temp[0];
			V_[i][1] = temp[1];
			robotV_ = verticesCounter_-2;
			i--;

			if(SHOW_LOG_INFO_)
				ROS_INFO_STREAM("Robot is entering the navigation graph at vertice #" << robotV_ );
		}
		else if(V_[i][0] == posGv_[0] && V_[i][1] == posGv_[1] && goalV_ < 0)
		{
			int temp[2] = {V_[verticesCounter_-1][0],V_[verticesCounter_-1][1]};
			V_[verticesCounter_-1][0] = V_[i][0];
			V_[verticesCounter_-1][1] = V_[i][1];
			V_[i][0] = temp[0];
			V_[i][1] = temp[1];
			goalV_ = verticesCounter_-1;
			i--;

			if(SHOW_LOG_INFO_)
				ROS_INFO_STREAM("Robot is leaving the navigation graph at vertice #" << goalV_ );
		}
	}






}


void RHCF::buildNavigationGraph(){

	if(SHOW_LOG_INFO_)
 		ROS_INFO("1st block");
	edgeCostMatrix_ = new double*[verticesCounter_];
	for(int i = 0; i < verticesCounter_; ++i)
		edgeCostMatrix_[i] = new double[verticesCounter_];

	if(SHOW_LOG_INFO_)
		ROS_INFO("2 block");
	// Navigation graph adjacency matrix
	int navigationGraph[verticesCounter_][verticesCounter_];
	// Cost (distances) matrix
	costMatrix_ = new double*[verticesCounter_];
	for(int i = 0; i < verticesCounter_; ++i)
		costMatrix_[i] = new double[verticesCounter_];


	if(SHOW_LOG_INFO_)
		ROS_INFO("3 block");

	// Struct to keep x-y coordinatels of all pixels that form an edge in the graph
	// --> float**** edgePaths = NULL;
	// Struct to keep all pixels that form an edge in the graph
	// --> edgePaths = new float***[verticesCounter_];
	edgePathsCells_ = new float***[verticesCounter_];

	for(int i = 0; i<verticesCounter_; i++)
	{
		// --> edgePaths[i] = new float**[verticesCounter_];
		edgePathsCells_[i] = new float**[verticesCounter_];
		for(int j = 0; j<verticesCounter_; j++)
		{
			edgePathsCells_[i][j] = new float*[2];
			edgePathsCells_[i][j][0] = new float[1];
			edgePathsCells_[i][j][1] = new float[1];
		}

	}


	// Lengths of the edges in pixels
	if(SHOW_LOG_INFO_)
		ROS_INFO("4 block");

	edgePathsLengths_ = new int*[verticesCounter_];
	for(int i = 0; i < verticesCounter_; ++i)
		edgePathsLengths_[i] = new int[verticesCounter_];

	for(int i = 0; i < verticesCounter_; ++i)
		for(int j = 0; j < verticesCounter_; ++j)
		{
			edgePathsLengths_[i][j] = 0;
		}
	
	if(SHOW_LOG_INFO_)
		ROS_INFO("5 block");

	bool pairOfVertices = false;

	for(int i = 0; i<verticesCounter_; i++)
		for(int j = 0; j<verticesCounter_; j++)
		{

			navigationGraph[i][j] = 0;
			costMatrix_[i][j] = 0;
			edgeCostMatrix_[i][j] = 0;
			edgePathsLengths_[i][j] = 0;

			if(abs(V_[i][0] - V_[j][0]) + abs(V_[i][1] - V_[j][1]) == 1) pairOfVertices = true;
		}

	// Current vertice identifier
	// Start random walk from the ii vertice, end up in another vertex
	// Add the edge between them
	int ii = 0;
	
	if(SHOW_LOG_INFO_)
		ROS_INFO("6 block");

	clock_t NAVtimeStart = clock();	

	while(true)
	{

		if(ii>=verticesCounter_) break;
		// Skip current vertice if it has no neighbors

		if(voronoiArray_[V_[ii][0]-1][V_[ii][1]] != 1 && voronoiArray_[V_[ii][0]+1][V_[ii][1]] != 1 &&
			voronoiArray_[V_[ii][0]][V_[ii][1]-1] != 1 && voronoiArray_[V_[ii][0]][V_[ii][1]+1] != 1)
		{
			ii++;
			continue;
		}

		//Initialize current node of the random walk with the current vertex
		int currentNode[2] = {V_[ii][0], V_[ii][1]};

		// Structs to keep the edge pixel paths as (i,j) and (x,y)
		vector<int> pathCells[2];

		pathCells[0].push_back(V_[ii][0]);
		pathCells[1].push_back(V_[ii][1]);

		int counter = 0;
		double cost = 0;
		
		ROS_INFO("ii %d", ii);

		while(true)
		{

			clock_t NAVtimeEnd = clock();
			size_t NAVtime = 1000 * (NAVtimeEnd - NAVtimeStart) / CLOCKS_PER_SEC;

			if(NAVtime > timeout_)
			{
				ROS_WARN_STREAM("\n[WARNING] Timeout of " << timeout_ << " milliseconds reached. Navigation graph construction halted.\n\n");

				break;
			}

			// Check if we go close to the borders
			if(	currentNode[0] == 1 || currentNode[0] == (sizeX_-2) || currentNode[1] == 1 || currentNode[1] == (sizeY_-2))
				{

					ii = 0;
					int npoints = (int )pathCells[0].size();

					for (int i_p =1; i_p < npoints; i_p++){
							int x_i = 	pathCells[0][i_p];
							int y_i = 	pathCells[1][i_p];
							aImage_(x_i,y_i) = 255 - (255*socialCostMatrix_[x_i][y_i])/maxSF_;
					}

					pathCells[0].clear();
					pathCells[1].clear();
					break;
				}
			// cout << "Current node " << currentNode[0] << ", " << currentNode[1] << , << " counter "<< counter << endl;
			voronoiArray_[currentNode[0]][currentNode[1]] = 0;
			

			if(voronoiArray_[currentNode[0]-1][currentNode[1]] == 1 && counter == 0)
			{
				currentNode[0]--;
			}
			else if(voronoiArray_[currentNode[0]+1][currentNode[1]] == 1 && counter == 0)
			{
				currentNode[0]++;
			}
			else if(voronoiArray_[currentNode[0]][currentNode[1]-1] == 1 && counter == 0)
			{
				currentNode[1]--;
			}
			else if(voronoiArray_[currentNode[0]][currentNode[1]+1] == 1 && counter == 0)
			{
				currentNode[1]++;
			}
			else if(voronoiArray_[currentNode[0]-1][currentNode[1]] >= 1)
			{
				currentNode[0]--;
			}
			else if(voronoiArray_[currentNode[0]+1][currentNode[1]] >= 1)
			{
				currentNode[0]++;
			}
			else if(voronoiArray_[currentNode[0]][currentNode[1]-1] >= 1)
			{
				currentNode[1]--;
			}
			else if(voronoiArray_[currentNode[0]][currentNode[1]+1] >= 1)
			{
				currentNode[1]++;
			}
			else{
				ii = -1;
				break;

			}
			counter++;

			pathCells[0].push_back(currentNode[0]);
			pathCells[1].push_back(currentNode[1]);

			if(voronoiArray_[currentNode[0]][currentNode[1]] == 2)
				break;
			else
				cost += socialCostMatrix_[currentNode[0]][currentNode[1]];
		}

		if( ii > -1) {

				int finalNode = -1;

				// Identify the final node (i,j) -> vertex ID
				for(int k = 0; k<verticesCounter_; k++)
				{
					voronoiArray_[V_[k][0]][V_[k][1]] = 2;

					if(currentNode[0] == V_[k][0] && currentNode[1] == V_[k][1])
						finalNode = k;
				}

				// Unlikely situation, then two vertices have multiple edges between them
				if(navigationGraph[ii][finalNode] == 1)
				{
					ROS_WARN_STREAM("\n[WARNING] Multiple edges found.\n\n");

					//return 1;
				}

				navigationGraph[ii][finalNode] = 1;
				navigationGraph[finalNode][ii] = 1;


				// costMatrix_[ii][finalNode] = counter;
				// costMatrix_[finalNode][ii] = counter;

				costMatrix_[ii][finalNode] = alpha_*cost + (1-alpha_)*counter*sqrt(scene_->CELLWIDTH_*scene_->CELLHEIGHT_);
				costMatrix_[finalNode][ii] = alpha_*cost + (1-alpha_)*counter*sqrt(scene_->CELLWIDTH_*scene_->CELLHEIGHT_);

				if(SHOW_LOG_INFO_)
					ROS_INFO_STREAM("Cost = " << cost << ", length = " << counter << ", aplha = " << alpha_ << "sqrt = " << sqrt(scene_->CELLWIDTH_*scene_->CELLHEIGHT_));

				edgePathsLengths_[ii][finalNode] = counter;
				edgePathsLengths_[finalNode][ii] = counter;
				
				alpha_ = 0.5;
				
				edgeCostMatrix_[ii][finalNode] = alpha_*cost + (1-alpha_)*counter*sqrt(scene_->CELLWIDTH_*scene_->CELLHEIGHT_);
				edgeCostMatrix_[finalNode][ii] = alpha_*cost + (1-alpha_)*counter*sqrt(scene_->CELLWIDTH_*scene_->CELLHEIGHT_);

				delete [] edgePathsCells_[ii][finalNode][0];
				delete [] edgePathsCells_[ii][finalNode][1];
				delete [] edgePathsCells_[finalNode][ii][0];
				delete [] edgePathsCells_[finalNode][ii][1];
				edgePathsCells_[ii][finalNode][0] = new float[counter];
				edgePathsCells_[ii][finalNode][1] = new float[counter];
				edgePathsCells_[finalNode][ii][0] = new float[counter];
				edgePathsCells_[finalNode][ii][1] = new float[counter];

				for(int k = 0; k<counter; k++)
				{

					edgePathsCells_[ii][finalNode][0][k] = pathCells[0].at(k);
					edgePathsCells_[ii][finalNode][1][k] = pathCells[1].at(k);
					edgePathsCells_[finalNode][ii][0][k] = pathCells[0].at(counter-k);
					edgePathsCells_[finalNode][ii][1][k] = pathCells[1].at(counter-k);
				}

	 	}


		ii = 0;
	}
	
	if(SHOW_LOG_INFO_)
		ROS_INFO("7 block");

	if(pairOfVertices)
	{
		for(int i = 0; i<verticesCounter_; i++)
			for(int j = i; j<verticesCounter_; j++)
				if(abs(V_[i][0] - V_[j][0]) + abs(V_[i][1] - V_[j][1]) == 1)
				{
					navigationGraph[i][j] = 1;
					navigationGraph[j][i] = 1;

					costMatrix_[i][j] = 1;
					costMatrix_[j][i] = 1;

					edgePathsLengths_[i][j] = 1;
					edgePathsLengths_[j][i] = 1;


					edgePathsCells_[i][j][0][0] = V_[i][0];
					edgePathsCells_[i][j][1][0] = V_[i][1];
					edgePathsCells_[j][i][0][0] = V_[j][0];
					edgePathsCells_[j][i][1][0] = V_[j][1];

					ROS_WARN_STREAM("\n[WARNING] Found 2 neighboring vertices.\n\n");

				}
	}

	if(SHOW_LOG_INFO_)
		ROS_INFO("7 block");

	int neighborsCounter[verticesCounter_];
	// Modify cost matrix
	for(int i = 0; i<verticesCounter_; i++)
	{
		neighborsCounter[i] = 0;
		for(int j = 0; j<verticesCounter_; j++)
			if(costMatrix_[i][j] > 0) neighborsCounter[i] += 1;
	}

	for(int i = 0; i<verticesCounter_; i++)
		if(neighborsCounter[i] > 2)
			for(int j = 0; j<verticesCounter_; j++)
				if(costMatrix_[i][j] > 0 && neighborsCounter[j] == 2)
					for(int k = 0; k<verticesCounter_; k++)
					{
						if(k!=i)
						{
							costMatrix_[i][j] += costMatrix_[j][k];
							if(costMatrix_[j][k] > 0 && neighborsCounter[k] == 2)
								for(int kk = 0; kk<verticesCounter_; kk++)
								{
									if(kk!=i && kk!= j)
									{
										costMatrix_[i][j] += costMatrix_[k][kk];
										if(costMatrix_[k][kk] > 0 && neighborsCounter[kk] == 2)
											for(int kkk = 0; kkk<verticesCounter_; kkk++)
											{
												if(kkk!=i && kkk!= j && kkk != k)
												{
													costMatrix_[i][j] += costMatrix_[kk][kkk];
												}
											}
									}
								}
						}
					}

	if(SHOW_LOG_INFO_)
		ROS_INFO("8 block");




		aImageFlipped_ = (aImage_);
		for(int i = 0; i < (aImage_).xSize(); i++)
			for(int j = 0; j < (aImage_).ySize(); j++)
				aImageFlipped_(i,j) = (aImage_)(i,(aImage_).ySize() - 1 - j);

		aImageFlipped_.writeToPGM("images/navigationGraph.pgm");
	

	


}

void RHCF::findHomotopyClasses(){
	ROS_INFO_STREAM("How many paths? do you need Sir? "<<K_);
	no_path_found_ = false;

	for(int i = 0; i < verticesCounter_; i++)
	{
		double rowSum = 0;
		for(int j = 0; j < verticesCounter_; j++)
		{
			if(costMatrix_[i][j] != 0) costMatrix_[i][j] = 1/(costMatrix_[i][j]);
			rowSum+=costMatrix_[i][j];
		}
		if(rowSum > 0)
			for(int j = 0; j < verticesCounter_; j++)
				costMatrix_[i][j]/=rowSum;
	}


	/// TODO: check filenames!
	if(SHOW_LOG_INFO_)
	{


		ofstream probs;
		ofstream costs;
	  	costs.open ("costMatrix.txt");
	  	probs.open ("probMatrix.txt");
	  		// Print navigation graph
			ROS_INFO_STREAM("Updated cost matrix:\n");


		for(int i = 0; i<verticesCounter_; i++)
		{
			for(int j = 0; j<verticesCounter_; j++)
			{
					cout << costMatrix_[i][j] << "\t";
					probs << costMatrix_[i][j] << "\t";

					costs << edgeCostMatrix_[i][j] << "\t";
			}

					cout << endl;
					costs << "\n";
					probs << "\n";
		}
		

		probs.close();
		costs.close();

		costs.open ("nodesXY.txt");
		
		for(int i = 0; i<verticesCounter_; i++)
			costs << scene_->XMIN_ + V_[i][0] * scene_->CELLWIDTH_ << "\t" << scene_->YMIN_ + V_[i][1] * scene_->CELLHEIGHT_ << "\n";
		
		costs.close();

	}


	std::vector<int> sparsCostMatrix[verticesCounter_];



	double initialCostMatrix[verticesCounter_][verticesCounter_];
	for(int i = 0; i < verticesCounter_; i++)
		 for(int j = 0; j < verticesCounter_; j++)
		 	{
		 		initialCostMatrix[i][j] = costMatrix_[i][j];

		 		if(initialCostMatrix[i][j]>0)
		 			sparsCostMatrix[i].push_back(j);


		 	}

	numHC_ = 0;

	int** HCpaths = new int*[K_];
	int HCpathsLengths[K_];
//	bool neighbourPath = false;

	int counter = 0;

	srand (time(NULL));

	clock_t HCtimeStart = clock();

// Run random walk iterations until K distinct paths are found
while (numHC_ < K_ )
	{
		//numHC_++;
		counter++;


		for(int i = 0; i < verticesCounter_; i++)
		 	for(int j = 0; j < sparsCostMatrix[i].size(); j++)
		 		costMatrix_[i][ sparsCostMatrix[i].at(j) ] = initialCostMatrix[i][ sparsCostMatrix[i].at(j) ];

		int currentNode = robotV_;
		// goalV_

		vector<int> path;
		path.push_back(currentNode);

		while(true)
		{
			//ROS_INFO("INNER WHILE LOOP");

			vector<int> currentNodeNeighbours;
			vector<double> currentNodeNeighboursProbs;
			double cumSumTotal = 0;

			for(int i = 0; i < sparsCostMatrix[currentNode].size() ; i++)
			{
				if(costMatrix_[currentNode][ sparsCostMatrix[currentNode].at(i) ] > 0)
				{
					currentNodeNeighbours.push_back( sparsCostMatrix[currentNode].at(i) );
					currentNodeNeighboursProbs.push_back(costMatrix_[currentNode][sparsCostMatrix[currentNode].at(i)]  );
					cumSumTotal+=costMatrix_[currentNode][sparsCostMatrix[currentNode].at(i)];
				}
			}



			if(currentNodeNeighbours.size() > 0)
			{
//				int randomNode = rand() % currentNodeNeighbours.size();
				double randomNum = (rand() % 10000);
				randomNum/=10000;


				// if(SHOW_LOG_INFO_)
				// 	ROS_INFO_STREAM("Random number = " << randomNum );

				int randomNode = 0;
				double cumSum = 0;
				for(int i = 0; i < currentNodeNeighbours.size(); i++)
				{
					cumSum+=(currentNodeNeighboursProbs[i]/cumSumTotal);

					// if(SHOW_LOG_INFO_)
					// 	ROS_INFO_STREAM("Cum Sum = " << cumSum);

					if(cumSum>randomNum)
					{
						randomNode = i;
						break;
					}
				}

								// DF discountation factor
				//double df = 0.1;
				//Multiply the last used edge by DF, consequently, deareasing the probability
				initialCostMatrix[currentNode][currentNodeNeighbours[randomNode]] *= df_;
				initialCostMatrix[currentNodeNeighbours[randomNode]][currentNode] *= df_;


				//cout << "RANDOM NODE #" << randomNode << endl;
				path.push_back(currentNodeNeighbours[randomNode]);
				for(int i = 0; i < verticesCounter_; i++)
				{
					if(costMatrix_[currentNode][i] > 0)
					{
						// costMatrix_[currentNode][i] = 0;
						costMatrix_[i][currentNode] = 0;
					}
					// Divide each non-used edge from currentNode by the DF, consequently, increasing the probability
					if(initialCostMatrix[currentNode][i] > 0 && i != currentNodeNeighbours[randomNode])
					{
						initialCostMatrix[currentNode][i] /= df_;
						initialCostMatrix[i][currentNode] /= df_;
					}

				}
				currentNode = currentNodeNeighbours[randomNode];
			}
			else
			{

				break;
			}

			if(currentNode == goalV_)
			{	
				int pathlength = (int)path.size();
				initialCostMatrix[path.at(pathlength-2)][path.at(pathlength-1)] /= df_;
				initialCostMatrix[path.at(pathlength-1)][path.at(pathlength-2)] /= df_;
				break;

			}
		}

		if(path[path.size()-1] == goalV_)
		{
			bool isPathNew = true;
			for (int i = 0; i < numHC_; i++)
			{
				if(path.size() != HCpathsLengths[i])
					continue;


				int temp = 0;
				for (int j = 0; j < path.size(); j++)
					if(path[j] == HCpaths[i][j])
						temp++;
					else break;
				if(temp == path.size())
				{
					isPathNew = false;
					break;
				}
			}
			if(isPathNew)
			{
				HCpaths[numHC_] = new int[path.size()];
				HCpathsLengths[numHC_] = path.size();
				for(int i = 0; i < path.size(); i++)
					HCpaths[numHC_][i] = path[i];
				numHC_++;
			}
		}
		clock_t HCtimeEnd = clock();
		size_t HCtime = 1000 * (HCtimeEnd - HCtimeStart) / CLOCKS_PER_SEC;
		if(HCtime > timeout_)
		{
			ROS_WARN_STREAM("\n[WARNING] Timeout of " << timeout_ << " milliseconds reached. Homotopy class search halted.\n\n");

			break;
		}
	}


	int HCpathsCellLengths[numHC_];
	double HCpathsCellCosts[numHC_];
	int shortestPathID = 0;
	int bestPathID = 0;
	double avgLength = 0; /// TODO remove it

	for (int i = 0; i < numHC_; i++)
	{
		ROS_INFO_STREAM( "path #" << i << ", cost = ");
		HCpathsCellLengths[i] = 0;
		HCpathsCellCosts[i] = 0;
		for(int j = 0; j < HCpathsLengths[i]-1; j++)
		{
			HCpathsCellLengths[i] +=edgePathsLengths_[HCpaths[i][j]][HCpaths[i][j+1]];
			HCpathsCellCosts[i]+=edgeCostMatrix_[HCpaths[i][j]][HCpaths[i][j+1]];
			cout << edgeCostMatrix_[HCpaths[i][j]][HCpaths[i][j+1]] << " + ";
		}
		ROS_INFO_STREAM("\n\n\tTotal cost = " << HCpathsCellCosts[i] << "\tTotal length = " << HCpathsCellLengths[i]*0.1);

		avgLength += HCpathsCellLengths[i]; /// TODO remove it

		if(i==0)
		{
			shortestPathID = i;
			bestPathID = i;
		}
		else
		{
			if(i>0 && HCpathsCellLengths[i] < HCpathsCellLengths[shortestPathID]) shortestPathID = i;
			if(i>0 && HCpathsCellCosts[i] < HCpathsCellCosts[bestPathID]) bestPathID = i;
		}
	}

	if(numHC_>0)
		avgLength/=numHC_; /// TODO remove it
	else{
		ROS_INFO("No path was found!!!");
		no_path_found_ = true;
		avgLength=0; /// TODO remove it
    	return;

	}
	ROS_INFO_STREAM("Average length = " << avgLength );
	ROS_INFO_STREAM("Shortest path: " << shortestPathID );
	ROS_INFO_STREAM("Best path: " << bestPathID);

	/// TODO check if done properly
	/// save best path
	best_path_.clear();

	for(int j = 0; j < HCpathsLengths[bestPathID]-1; j++)
	{
		for(int k = 0; k < edgePathsLengths_[HCpaths[bestPathID][j]][HCpaths[bestPathID][j+1]]; k++)
		{
			int v1 = edgePathsCells_[HCpaths[bestPathID][j]][HCpaths[bestPathID][j+1]][0][k];
			int v2 = edgePathsCells_[HCpaths[bestPathID][j]][HCpaths[bestPathID][j+1]][1][k];

   			geometry_msgs::Point p;
			p.x = scene_->XMIN_ + scene_->CELLWIDTH_*v1 ;
			p.y = scene_->YMIN_ + scene_->CELLHEIGHT_* (v2) ;
			p.z = 0;

			best_path_.push_back(p);

		}
	}


	//cout << "Found paths in the end: " << endl;
	for(int i = 0; i < numHC_; i++)
	{
		//cout << "Drawing path number " << i << endl;
		//cout << "Path length = " << HCpathsLengths[i] << endl;
		
		CMatrix<float> pathImage;
		
		if(SHOW_LOG_INFO_)
			pathImage = aImageFlipped_;
		
		std::vector<RealPoint> pathi;
		
		for(int j = 0; j < HCpathsLengths[i]-1; j++)
		{
			//cout << "Edge = " << HCpaths[i][j] << " - " << HCpaths[i][j+1] << endl;
			//cout << "V [" << HCpaths[i][j] << "] = " << V[HCpaths[i][j]][0] << "," << V[HCpaths[i][j]][1] << endl;

			for(int k = 0; k < edgePathsLengths_[HCpaths[i][j]][HCpaths[i][j+1]]; k++)
			{
				int v1 = edgePathsCells_[HCpaths[i][j]][HCpaths[i][j+1]][0][k];
				int v2 = edgePathsCells_[HCpaths[i][j]][HCpaths[i][j+1]][1][k];
				// cout << v1 << "," << v2 << endl;
	 			if(SHOW_LOG_INFO_)
					pathImage(v1,(aImage_).ySize() - v2 - 1) = 100;

				double xp = scene_->XMIN_ + scene_->CELLWIDTH_*v1 ;
				double yp = scene_->YMIN_ + scene_->CELLHEIGHT_*v2 ;

				pathi.push_back(RealPoint(xp,yp));
				// ROS_INFO("Storing point %f %f", xp, yp);
			}
		
			if(SHOW_LOG_INFO_){

			pathImage.drawLine(posR_[0],(aImage_).ySize()-1-posR_[1],posRV_[0],(aImage_).ySize()-1-posRV_[1], 100);
			pathImage.drawLine(posG_[0],(aImage_).ySize()-1-posG_[1],posGv_[0],(aImage_).ySize()-1-posGv_[1], 100);

			}

		}


	if(SHOW_LOG_INFO_)
		ROS_INFO("New PAth ");

		paths_.push_back(pathi);

		string animationNum;
		if(i<10) animationNum = "00" + to_string(i);
		else if(i<100) animationNum = "0" + to_string(i);
		else animationNum = to_string(i);
		const char * fName = (string("images/path") + animationNum + string(".pgm")).c_str();
		//if (i == shortestPathID) pathImage.writeToPGM(fName);
	
	if(SHOW_LOG_INFO_)
		pathImage.writeToPGM(fName);
		//cout << endl;
	}

	
	
	if(SHOW_LOG_INFO_){
		
		double *res_diversity_robdiversity;
		res_diversity_robdiversity = measureDiversity(HCpathsCellLengths,HCpathsLengths, HCpaths);
		ROS_INFO("diversity %f, robustDiversity %f", res_diversity_robdiversity[0], res_diversity_robdiversity[1]);

	}

	for(int i = 0; i < numHC_; i++)
	{
		delete [] HCpaths[i];
	}

	delete [] HCpaths;


}
