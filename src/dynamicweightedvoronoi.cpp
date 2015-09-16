#include <srl_rhcf_planner/dynamicvoronoi/dynamicweightedvoronoi.h>

#include <math.h>
#include <iostream>

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  type_weight_ = 0;
  weighting_map_ = NULL;
  alternativeDiagram = NULL;
  allocatedGridMap = false;
  social_gain_ = 2;
  border_gain_ = 5;
  social_cost_file.open ("images/SocialDistanceMatrix.txt");

}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (allocatedGridMap && gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }

  if(weighting_map_){

    for (int x=0; x<sizeX; x++){
     delete[] weighting_map_[x];
     delete[] dist_closest_humans_[x];
    }
    delete[] dist_closest_humans_;
    delete[] weighting_map_;

  }

  social_cost_file.close();
}


void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
    data = NULL;
  if(alternativeDiagram){
    for (int x=0; x<sizeX; x++) delete[] alternativeDiagram[x];
    delete[] alternativeDiagram;
    alternativeDiagram = NULL;
  }
  if (initGridMap) {
    if (allocatedGridMap && gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
      gridMap = NULL;
      allocatedGridMap = false;
    }
  }


  sizeX = _sizeX;
  sizeY = _sizeY;
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];



  if (initGridMap) {
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
    allocatedGridMap = true;


  }

  dataCell c;
  c.dist = INFINITY;
  c.gridDist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

  if (initGridMap) {
    for (int x=0; x<sizeX; x++)
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}



void DynamicVoronoi::initializeWeightingMap(int _sizeX, int _sizeY, int** _weightingMap) {

  weighting_map_ = new int*[_sizeX];
  for (int x=0; x<_sizeX; x++) weighting_map_[x] = new int[_sizeY];


  for(int x=0; x<_sizeX; x++){
    for(int y=0; y<_sizeY; y++){

      weighting_map_[x][y] = 0;
    }
  }

  for(int x=0; x<_sizeX; x++){
    for(int y=0; y<_sizeY; y++){

      weighting_map_[x][y] = 255 - _weightingMap[x][y];
    }
  }



}

void DynamicVoronoi::initializeHumanInfo(int _sizeX, int _sizeY, std::vector<humanCell> hinfo){

  humans_data.clear();


  int nh = (int)hinfo.size();

  for(int i=0; i<nh; i++){
    humanCell h = hinfo[i];
    humans_data.push_back(h);
  }



  h_agents_data = new humanCell*[sizeX];
  for (int x=0; x<sizeX; x++) h_agents_data[x] = new humanCell[sizeY];

  /// setting to zero
  for(int x=0; x<_sizeX; x++){
    for(int y=0; y<_sizeY; y++){
      humanCell h;
      h.x = x;
      h.y = y;
      h.vx = 0;
      h.vy = 0;
      h_agents_data[x][y] = h;
    }
  }

  /// Filling it with the proper info
  for(int i=0; i<nh; i++){
    humanCell h = hinfo[i];

    h_agents_data[h.x][h.y].vx = h.vx ;
    h_agents_data[h.x][h.y].vy = h.vy ;
  }

  

  dist_closest_humans_ = new float*[_sizeX];
  for (int x=0; x<_sizeX; x++) dist_closest_humans_[x] = new float[_sizeY];

  for(int x=0; x<_sizeX; x++){
    for(int y=0; y<_sizeY; y++){
        
        dist_closest_humans_[x][y] = INFINITY ;


        for(int i=0; i<nh; i++){
             humanCell h = hinfo[i];

             float dist = (h.x-x)*(h.x-x) + (h.y-y)*(h.y-y);

             if(dist < dist_closest_humans_[x][y]){
              
                dist_closest_humans_[x][y] = dist;

             }
      
      
        }
    }
  }

}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {

          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.gridDist = 0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;

  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT>& points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }

  lastObstacles.clear();

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }
}

void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue;

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) {nc.dist = INFINITY; nc.gridDist= INFINITY;}
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            /// TODO: distance to be computed differently or weighted
            // int distx = nx-c.obstX;
            // int disty = ny-c.obstY;
            // int newSqDistance = distx*distx + disty*disty;
            int newSqDistance = computeDistance( nx, ny, c.obstX, c.obstY);

            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) {
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny)); /// TODO: change the queue to accept float
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);

                int distx = nx-c.obstX;
                int disty = ny-c.obstY;
                int newGridSqDistance = distx*distx + disty*disty;
                nc.gridDist = sqrt((double) newGridSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else {
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}


//! Select type of weighing
void DynamicVoronoi::setTypeWeighting(int t){

  type_weight_ = t;

}




void DynamicVoronoi::settingGains(double social, int border, double direction_gain){

    social_gain_ = social ;
    border_gain_ = border;
    direction_gain_ = direction_gain;

    std::cout<<"Setting gains, Social : "<< social_gain_ << ", Border : " << border_gain_<< " Direction Gain :" << direction_gain_ <<std::endl;

}


//! compute squared distance between two cells
// TODO insert weight or compute distance in a different way
int DynamicVoronoi::computeDistance(int xa, int ya, int xb, int yb){

  // a is the point, b is obstacle

  int distx = (xb-xa);
  int disty = (yb-ya);
  double sqrtDist = sqrt(distx*distx + disty*disty);
  int valueA = ( weighting_map_[xa][ya] );
  int valueB = ( weighting_map_[xb][yb] );

  double  weightDist = 0;
  weightDist = ( ( fabs( (double)valueB - (double)valueA) ) ) / direction_gain_ ;

  double socialCost = 0;





  int newSqDistance = 0;

  // Euclidean Distance on the grid
  if(type_weight_ == 0){

    newSqDistance = (int)(distx*distx + disty*disty) ;

  }


  // additive weighting of Euclidean distance on the grid, it generates connected regions..
  if(type_weight_ == 1){

    newSqDistance = ( - weightDist) + (distx*distx + disty*disty);

  }


  // multipl weighting of Euclidean distance on the grid, it generates not connected regions
  if(type_weight_ == 2){

    newSqDistance = ( distx*distx + disty*disty ) / (( valueA + 1)) ;

  }


  // Only Considering heading angle of the agents
  if(type_weight_ == 3){
    // rotating of 90 degrees
    double x =  + h_agents_data[xb][yb].vy;
    double y = -h_agents_data[xb][yb].vx ;

    // newSqDistance =   + (distx*distx + disty*disty) + (fabs( direction_gain_*h_agents_data[xb][yb].vx * distx + direction_gain_*h_agents_data[xb][yb].vy * disty ));
    newSqDistance =   (int) round( (distx*distx + disty*disty) + (fabs( + direction_gain_*x * distx + direction_gain_*y * disty )) );

  }

  // Considering Anisotropic social cost
  // Additive social model
  if(type_weight_ == 4){
    // Consider now a cost related to the social field
    if(sqrtDist == 0 )
    {

      return (distx*distx + disty*disty) ;

    }

    // handling cases where we are out of the grid
    if ( (xb == 0) || (yb == 0) || (xb == (sizeX-1) ) || (yb == (sizeY-1) )) {
      return (distx*distx + disty*disty) ;

    }

    double distxn = distx/sqrtDist;
    double distyn = disty/sqrtDist;

    if(distxn>=0 && distyn>=0)
        socialCost = ( distxn*distxn* (weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));
    else  if(distxn>=0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn>=0)
          socialCost = ( distxn*distxn* (weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));

    newSqDistance = ( sqrt(distx*distx + disty*disty) + socialCost*( social_gain_/255.0) * sqrt(distx*distx + disty*disty) )*( sqrt(distx*distx + disty*disty) + socialCost*(social_gain_/255.0) * sqrt(distx*distx + disty*disty) );

  }



  // Considering Anisotropic social cost plus term related to direction of the agents
  // Power diagram social model + anisotropic weighting
  if(type_weight_ == 5){
    // Consider now a cost related to the social field
    if(sqrtDist == 0 )
    {

      return (distx*distx + disty*disty) ;

    }

    // handling cases where we are out of the grid
    if ( (xb == 0) || (yb == 0) || (xb == (sizeX-1) ) || (yb == (sizeY-1) )) {
      return (distx*distx + disty*disty) ;

    }

    double distxn = distx/sqrtDist;
    double distyn = disty/sqrtDist;

    if(distxn>=0 && distyn>=0)
        socialCost = ( distxn*distxn* (weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));
    else  if(distxn>=0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn>=0)
          socialCost = ( distxn*distxn* (weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));


    double x =  + h_agents_data[xb][yb].vy;
    double y =  - h_agents_data[xb][yb].vx ;

    newSqDistance =  (distx*distx + disty*disty) +
                    (fabs( direction_gain_*x * distx + direction_gain_*y * disty )) + 
                    socialCost*(social_gain_/255.0) * sqrt(distx*distx + disty*disty);

  }


  // Considering Anisotropic social cost, power diagram
  // Power diagram social model
  if(type_weight_ == 6){
    // Consider now a cost related to the social field
    if(sqrtDist == 0 )
    {

      return (distx*distx + disty*disty) ;

    }

    // handling cases where we are out of the grid
    if ( (xb == 0) || (yb == 0) || (xb == (sizeX-1) ) || (yb == (sizeY-1) )) {
      return (distx*distx + disty*disty) ;

    }

    double distxn = distx/sqrtDist;
    double distyn = disty/sqrtDist;

    if(distxn>=0 && distyn>=0)
        socialCost = ( distxn*distxn* (weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));
    else  if(distxn>=0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn>=0)
          socialCost = ( distxn*distxn* (weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));

    newSqDistance = ( (distx*distx + disty*disty) + socialCost*(social_gain_/255.0) * sqrt(distx*distx + disty*disty) );

  }

  // Considering Anisotropic social cost
  // Additive social model + anisotropic weights
  if(type_weight_ == 7){
    // Consider now a cost related to the social field
    if(sqrtDist == 0 )
    {
      return 0;
    }

    int nagents = (int)humans_data.size();
    
    if(nagents == 0){

        return sqrtDist;

    }
    double distxn = distx/sqrtDist;
    double distyn = disty/sqrtDist;

    double x =  + h_agents_data[xb][yb].vy ;
    double y =  - h_agents_data[xb][yb].vx ;

    // handling cases where we are out of the grid
    // if ( (xb == 0) || (yb == 0) || (xb == (sizeX-1) ) || (yb == (sizeY-1) )) {
      
    if ( x == 0 && y == 0) {

      return (int)round(border_gain_*(distx*distx + disty*disty)) ;
    
    }



    if(distxn>=0 && distyn>=0)
        socialCost = ( distxn*distxn* (weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));
    else  if(distxn>=0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb+1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn<0)
          socialCost = ( distxn*distxn*(weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb-1]));
    else  if(distxn<0 && distyn>=0)
          socialCost = ( distxn*distxn* (weighting_map_[xb-1][yb]) + distyn*distyn*(weighting_map_[xb][yb+1]));

    social_cost_file<<xb <<" " <<yb <<" "<<socialCost<<" "<< direction_gain_ << " "<< social_gain_<< " " << border_gain_<<std::endl;


    newSqDistance = (int) round( ( sqrt(distx*distx + disty*disty) + socialCost*(social_gain_/255.0) * sqrt(distx*distx + disty*disty) )*
                    ( sqrt(distx*distx + disty*disty) + socialCost*(social_gain_/255.0) * sqrt(distx*distx + disty*disty) ) + 
                    (fabs( direction_gain_*x * distx + direction_gain_*y * disty )) );


  }

  return newSqDistance;
}
// ============================================================================================
/// line(GNode *successor,GNode *parent_node)
/// Check if there is a geometric line of sight bewtween the two node
/// ============================================================================================
double  DynamicVoronoi::costFromline(int x0, int y0, int y1, int x1){


        double value = 0;

        int dx,dy;

        dy=abs(y1-y0);
        dx=abs(x1-x0);

        int length = 0;

        if(dy > dx)
          length = dy;
        else
          length = dx;

        int x,y;

        double t=0;

        for (int i=0; i< length; i++){

            t = double(i)/length;

            x = (int) round(x0 * (1.0-t) + x1 * t);
            y = (int) round(y0 * (1.0-t) + y1 * t);
            value += 255 - weighting_map_[x][y];
        }


        return value;
}


float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist;
  else return -INFINITY;
}

float DynamicVoronoi::getHumanDistance(int x, int y){

  return (sqrt(dist_closest_humans_[x][y]));

}

float DynamicVoronoi::getGridDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].gridDist;
  else return -INFINITY;
}
bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}

bool DynamicVoronoi::isVoronoiAlternative(int x, int y) {
  int v = alternativeDiagram[x][y];
  return (v == free || v == voronoiKeep);
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) {c.dist = 0; c.gridDist = 0;}
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, INTPOINT(x,y));
    if (updateRealDist) {c.dist  = INFINITY; c.gridDist = INFINITY;}
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) {
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny
      /// TODO compute distance
      // int dxy_x = x-nc.obstX;
      // int dxy_y = y-nc.obstY;
      // int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;

      int sqdxy = computeDistance( x, y, nc.obstX, nc.obstY);

      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      /// TODO compute distance
      //compute dist from nx,ny to obstacle of x,y
      // int dnxy_x = nx - c.obstX;
      // int dnxy_y = ny - c.obstY;
      // int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int sqdnxy = computeDistance( nx, ny, c.obstX, c.obstY);

      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
  }
}


void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) {
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  // write ppm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n#\n");
  fprintf(F, "%d %d\n255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){
    for(int x = 0; x<sizeX; x++){
      unsigned char c = 0;
      if (alternativeDiagram!=NULL && (alternativeDiagram[x][y] == free || alternativeDiagram[x][y]==voronoiKeep)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if(isVoronoi(x,y)){
        fputc( 0, F );
        fputc( 0, F );
        fputc( 255, F );
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(sqrt(data[x][y].sqdist)*10);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}


void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    sortedPruneQueue.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) {
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        sortedPruneQueue.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    }
    if (x-2>=0 && l.voronoi==occupied) {
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        sortedPruneQueue.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    }
    if (y+2<sizeY && t.voronoi==occupied) {
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        sortedPruneQueue.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    }
    if (y-2>=0 && b.voronoi==occupied) {
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        sortedPruneQueue.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    }
  }


  while(!sortedPruneQueue.empty()) {
    INTPOINT p = sortedPruneQueue.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (sortedPruneQueue.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        sortedPruneQueue.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

void DynamicVoronoi::updateAlternativePrunedDiagramWithHumans(){


if(alternativeDiagram==NULL){
    alternativeDiagram = new int*[sizeX];
    for(int x=0; x<sizeX; x++){
      alternativeDiagram[x] = new int[sizeY];
    }
  }


  std::queue<INTPOINT> end_cells;
  BucketPrioQueue<INTPOINT> sortedPruneQueue;
  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      dataCell& c = data[x][y];
  alternativeDiagram[x][y] = c.voronoi;
  if(c.voronoi <=free){
    sortedPruneQueue.push(c.sqdist, INTPOINT(x,y));
    end_cells.push(INTPOINT(x, y));
  }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
  alternativeDiagram[x][y] = voronoiKeep;
  sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x,y));
  end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
  alternativeDiagram[x][y] = voronoiKeep;
  sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x,y));
  end_cells.push(INTPOINT(x, y));
      }
    }
  }


  while (!sortedPruneQueue.empty()) {
    INTPOINT p = sortedPruneQueue.pop();

    if (markerMatchAlternative(p.x, p.y)) {
      alternativeDiagram[p.x][p.y]=voronoiPrune;
    } else {
  alternativeDiagram[p.x][p.y]=voronoiKeep;
    }
  }

  // //delete worms
  while (!end_cells.empty()) {
    
    INTPOINT p = end_cells.front();
    
    end_cells.pop();
    
    float distToObst = getGridDistance( p.x, p.y );

    /// Please (distToObst <= 3) && ( getHumanDistance(p.x,p.y) > 10) used to cut worms which are close to obstacles
    /// but far from human beings. Note these values were generated after an informal validation
    if ((isVoronoiAlternative(p.x,p.y) && getNumVoronoiNeighborsAlternative(p.x, p.y) == 1) || (distToObst <= 1.5) || (distToObst <= 3) && ( getHumanDistance(p.x,p.y) > 10) || 
        (p.x == 0 || p.x == 1 || p.y == 0 || p.y == 1 || p.x == sizeX-1 || p.x == sizeX-2 || p.y == sizeY-1 || p.y == sizeY-2)) {
      alternativeDiagram[p.x][p.y] = voronoiPrune;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (!(dx || dy) || (dx && dy)) {
            continue;
          }
          int nx = p.x + dx;
          int ny = p.y + dy;
          if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
            continue;
          }
          if (isVoronoiAlternative(nx,ny)) {
            if (getNumVoronoiNeighborsAlternative(nx, ny) == 1) {
              end_cells.push(INTPOINT(nx, ny));
            }
          }
//          if (isVoronoiAlternative(nx,ny)) {
//            if (nx == 0 || nx == 1 || ny == 0 || ny == 1 || nx == sizeX-1 || nx == sizeX-2 || ny == sizeY-1 || ny == sizeY-2) {
//              end_cells.push(INTPOINT(nx, ny));
//            }
//          }
        }
      }
    }
  }







}

void DynamicVoronoi::updateAlternativePrunedDiagram() {


  if(alternativeDiagram==NULL){
    alternativeDiagram = new int*[sizeX];
    for(int x=0; x<sizeX; x++){
      alternativeDiagram[x] = new int[sizeY];
    }
  }


  std::queue<INTPOINT> end_cells;
  BucketPrioQueue<INTPOINT> sortedPruneQueue;
  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      dataCell& c = data[x][y];
  alternativeDiagram[x][y] = c.voronoi;
  if(c.voronoi <=free){
    sortedPruneQueue.push(c.sqdist, INTPOINT(x,y));
    end_cells.push(INTPOINT(x, y));
  }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
  alternativeDiagram[x][y] = voronoiKeep;
  sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x,y));
  end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for(int x=1; x<sizeX-1; x++){
    for(int y=1; y<sizeY-1; y++){
      if( getNumVoronoiNeighborsAlternative(x, y) >= 3){
  alternativeDiagram[x][y] = voronoiKeep;
  sortedPruneQueue.push(data[x][y].sqdist, INTPOINT(x,y));
  end_cells.push(INTPOINT(x, y));
      }
    }
  }


  while (!sortedPruneQueue.empty()) {
    INTPOINT p = sortedPruneQueue.pop();

    if (markerMatchAlternative(p.x, p.y)) {
      alternativeDiagram[p.x][p.y]=voronoiPrune;
    } else {
  alternativeDiagram[p.x][p.y]=voronoiKeep;
    }
  }

  // //delete worms
  while (!end_cells.empty()) {
    INTPOINT p = end_cells.front();
    end_cells.pop();

    if (isVoronoiAlternative(p.x,p.y) && getNumVoronoiNeighborsAlternative(p.x, p.y) == 1) {
      alternativeDiagram[p.x][p.y] = voronoiPrune;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (!(dx || dy) || (dx && dy)) {
            continue;
          }
          int nx = p.x + dx;
          int ny = p.y + dy;
          if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
            continue;
          }
          if (isVoronoiAlternative(nx,ny)) {
            if (getNumVoronoiNeighborsAlternative(nx, ny) == 1) {
              end_cells.push(INTPOINT(nx, ny));
            }
          }
        }
      }
    }
  }
}



bool DynamicVoronoi::markerMatchAlternative(int x, int y) {
// prune if this returns true

  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
//  int obstacleCount=0;
  int voroCount = 0;
  for (dy = 1; dy >= -1; dy--) {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++) {
      if (dx || dy) {
        nx = x + dx;
        int v = alternativeDiagram[nx][ny];
        bool b = (v <= free && v != voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (v <= free && !(dx && dy))
          voroCount++;
        i++;
      }
    }
  }

  /*
   * 5 6 7
   * 3   4
   * 0 1 2
   */

  {
    //connected horizontal or vertically to only one cell
    if (voroCount == 1 && (f[1] || f[3] || f[4] || f[6])) {
      return false;
    }

    // 4-connected
    if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
      return false;

    if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
      return false;

  }
  return true;
}

int DynamicVoronoi::getNumVoronoiNeighborsAlternative(int x, int y) {
  int count = 0;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
        continue;
      }

      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY) {
        continue;
      }
      if (alternativeDiagram[nx][ny]==free || alternativeDiagram[nx][ny]==voronoiKeep) {
        count++;
      }
    }
  }
  return count;
}



DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;



  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}
