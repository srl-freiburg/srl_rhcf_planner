#ifndef _SCENE_H_
#define _SCENE_H_
#include <iostream>

using namespace std;

class Scene {

public:
		Scene(){
			N_agents_ = 0;
			xrobot_[0] = 0;
			xrobot_[1] = 0;
			xrobot_[2] = 0;
			xgoal_[0] = 0;
			xgoal_[1] = 0;
			xpeople_ = NULL;
			CELLWIDTH_ = 0;
			CELLHEIGHT_ = 0 ;
			XMIN_ = 0;
			XMAX_ = 0;
			YMIN_ = 0;
			YMAX_ = 0;

		}

		~Scene(){

			for(int i = 0; i<N_agents_; i++)
				delete [] xpeople_[i];

			delete [] xpeople_;

		}

		bool setRobot(double x, double y, double theta){

			xrobot_[0] = x;
			xrobot_[1] = y;
			xrobot_[2] = theta;

			return true;

		}



		bool setGoal(double x, double y){

			xgoal_[0] = x;
			xgoal_[1] = y;

			return true;
		}





		bool setAgents(int N)
		{
			N_agents_ = N;

			try{

  				xpeople_ = new double*[N_agents_];

        		for(int i = 0; i<N_agents_; i++)
        			xpeople_[i] = new double[3];

  			}
  			catch (int e)
			{
			    cout << "An exception occurred. Exception Nr. " << e << '\n';
			    return false;
			}



        		return true;
		}


		bool setAgentCoordinate(int i, int j, double value){

			xpeople_[i][j] = value;
			return true;
		}


		bool setScene(double CELLWIDTH, double CELLHEIGHT, double XMIN, double XMAX, double YMIN, double YMAX){


		 	CELLWIDTH_ = CELLWIDTH;
			CELLHEIGHT_ = CELLHEIGHT;
			XMIN_ = XMIN;
			XMAX_ = XMAX;
			YMIN_ = YMIN;
			YMAX_ = YMAX;


			return true;
		}






		int N_agents_;
		double xrobot_[3];
		double xgoal_[2];
		double** xpeople_ ;

		double CELLWIDTH_ ;
		double CELLHEIGHT_ ;
		double XMIN_;
		double XMAX_;
		double YMIN_ ;
		double YMAX_ ;




};

#endif
