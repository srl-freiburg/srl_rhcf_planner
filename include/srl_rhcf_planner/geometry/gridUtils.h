#ifndef _GRIDUTILS_H_
#define _GRIDUTILS_H_

using namespace std;

class GridUtils {

public:
		GridUtils(){


		}

		~GridUtils(){


		}

		int xyToNode(double x, double y, int xSize, double xStep, double yStep, double xMin, double yMin)
		{
			int i = 0;
			int j = 0;
			double temp = xMin;
			while(temp+xStep <= x)
			{
				temp+=xStep;
				i++;
			}
			temp = yMin;
			while(temp+yStep <= y)
			{
				temp+=yStep;
				j++;
			}		
			return xSize * i + j;
		}

		int xyToI(double x, double y, int xSize, double xStep, double yStep, double xMin, double yMin)
		{
			int i = 0;
			double temp = xMin;
			while(temp+xStep <= x)
			{
				temp+=xStep;
				i++;
			}	
			return i;
		}

		int xyToJ(double x, double y, int xSize, double xStep, double yStep, double xMin, double yMin)
		{
			int j = 0;
			double temp = yMin;
			while(temp+yStep <= y)
			{
				temp+=yStep;
				j++;
			}		
			return j;
		}

		double nodeToX(int nodeID, int xSize, double xStep, double yStep, double xMin, double yMin)
		{	
			int i = nodeID%xSize; // check correction before was /
			double x = xStep*i;
			return xMin + x;
		}

		double nodeToY(int nodeID, int xSize, double xStep, double yStep, double xMin, double yMin)
		{	
			int j = nodeID%xSize;
			double y = yStep*j;
			return yMin + y;
		}


};

#endif
