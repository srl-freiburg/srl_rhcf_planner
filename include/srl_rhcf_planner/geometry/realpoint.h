
/*! A light-weight integer point with fields x,y */
class RealPoint {
public:
  RealPoint() : x(0), y(0) {}
  RealPoint(double _x, double _y) : x(_x), y(_y) {}
  double x,y;
};
