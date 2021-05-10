#pragma once
#include "air_lab2/histogram_grid.h"

class HistogramPolar{

public:
  HistogramPolar();

  /* [4] Alpha = 5 for a robot to successfully traverse a course with
  obstacles 2 meters apart with a speed of 1 m/s */


  HistogramPolar(int alpha);

  void update_polar(HistogramGrid* grid, double current_x, double current_y);
  double getSmoothedValue(int k, int l);
  void smooth_histogram(int l);

  int alpha, sectors, origo;
  std::vector<int> magnitudes;
};
