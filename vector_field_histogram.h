#pragma once
#include "air_lab2/histogram_polar.h"


class VectorFieldHistogram{

public:
  VectorFieldHistogram(int _threshold, double _robot_size, int _s_max, int alpha);

  int expand_valley(HistogramPolar* histogram_polar, int start_sector);
  int find_best_sector(std::vector<std::pair<int,int>> candidate_valleys, double target_direction, int sectors);
  double calculate_direction(HistogramPolar* histogram_polar, double target_direction, double driving_direction);

  /*[1]The VFH system needs a fine-tuned threshold only for the most challenging
  applications(e.g., travel at high speed and in densely cluttered environments).
  Under less demanding conditions, the system performs well even with an imprecisely set threshold.*/
private:
  int s_max = 18;
  int threshold = 0;
  double robot_size;
};
