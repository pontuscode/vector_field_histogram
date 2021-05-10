#include "air_lab2/histogram_grid.h"

HistogramGrid::HistogramGrid(){
  // Moving window dimensions in paper: 33*33 cells
  this->dimension = 33;
  // Cell resolution in paper: 10x10cm
  this->cell_resolution = 0.1;
  // This should give us a range of about 3*3 meters.
  this->active_region = std::vector<std::vector<int>>(this->dimension, std::vector<int>(this->dimension));
  // "Sets our position" in the middle of the x-axis
  this->origo = (this->dimension-1)/2;
}

HistogramGrid::HistogramGrid(int _dimension, double _resolution){
  this->dimension = _dimension;
  this->cell_resolution = _resolution;
  this->active_region = std::vector<std::vector<int>>(this->dimension, std::vector<int>(this->dimension, 0));
  this->origo = (this->dimension-1)/2;
}

void HistogramGrid::update_grid(double r, double theta){
  // Convert range measurement to number of cells in the histogram
  int cells = (int)(r/this->cell_resolution);
  // Calculate cartesian coordinates
  int y = (int)floor(cells * sin(theta));
  int x = (int)floor(cells * cos(theta));
  // Check if the coordinates are within our active region
  if(abs(y) < this->origo &&
    abs(x) < this->dimension){
     // Check if the obstacle is to our left (theta > 0) or right (theta < 0)
    if(theta < 0){
      this->active_region[this->origo + abs(y)][abs(x)] += 1;
    } else {
      this->active_region[this->origo - abs(y)][abs(x)] += 1;
    }
  }
}

std::vector<std::pair<int,int>> HistogramGrid::get_dangerzones(){
  std::vector<std::pair<int, int>> dangerCoords;
  for(int i = 0; i < this->active_region.size(); i++){
    for(int j = 0; j < this->active_region[i].size(); j++){
      dangerCoords.push_back(std::make_pair(i,j));
    }
  }
  return dangerCoords;
}
