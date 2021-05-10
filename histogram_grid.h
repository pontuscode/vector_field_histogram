#pragma once
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


#define PI 3.141592
class HistogramGrid {

public:
  HistogramGrid();
  HistogramGrid(int dimensions, double resolution);

  void update_grid(double r, double theta);

  std::vector<std::pair<int,int>> get_dangerzones();

  int getDimension(){ return this->dimension; }
  double getResolution() { return this->cell_resolution; }
  int getCell(int x, int y){ return this->active_region[x][y]; }
private:
  int dimension;
  double cell_resolution;
  int origo;
  /*
  In their implemention, the size of the window is 33 x 33 cells
  (with a  cell size of 10 cm X 10 cm), and the window is always centered about
  the robot's position.
  */
  std::vector<std::vector<int>> active_region;
};
