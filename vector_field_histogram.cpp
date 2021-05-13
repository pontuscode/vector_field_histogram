#include "air_lab2/vector_field_histogram.h"


VectorFieldHistogram::VectorFieldHistogram(int _threshold, double _robot_size,
                                           int _s_max, int _alpha){
  this->threshold = _threshold;
  // Number of sectors the robot occupies -> multiply it by alpha
  this->robot_size = _robot_size;
  this->s_max = _s_max;
}

int modulo(int x, int m) {
   /*Source: http://crsouza.blogspot.com/2009/09/modulo-and-modular-distance-in-c.html*/
  int r;
  if (m < 0){
    m = -m;
  }

  r = x % m;
  if(r < 0){
    return r + m;
  } else {
    return r;
  }
}

int modular_dist(int a, int b, int m) {
  int dist_a = modulo(a - b, m);
  int dist_b = modulo(b - a, m);
  if(dist_a < dist_b){
    return dist_a;
  } else {
    return dist_b;
  }
}


int VectorFieldHistogram::expand_valley(HistogramPolar* histogram_polar,
                                        int start_sector){
  int valley_size = 0;
  for(int k = start_sector; k < (start_sector + s_max); k++){
    if(k >= histogram_polar->sectors){
      break;
    }
    if(histogram_polar->magnitudes[k] < this->threshold){
      valley_size++;
    } else {
      break;
    }
  }
  return valley_size;
}

int VectorFieldHistogram::find_best_sector(std::vector<std::pair<int,int>> candidate_valleys,
                                           double target_direction, int sectors){
  int best_sector = -1;
  for(int valley = 0; valley < candidate_valleys.size(); valley++){
    int candidate_sector = candidate_valleys[valley].first + (candidate_valleys[valley].second/2);

    int dist_best_and_target = modular_dist(best_sector, target_direction, sectors);
    int dist_candidate_and_target = modular_dist(candidate_sector, target_direction, sectors);
    if(best_sector == -1 || dist_candidate_and_target < dist_best_and_target){
      best_sector = candidate_sector;
    }
  }
  return best_sector;
}

double VectorFieldHistogram::calculate_direction(HistogramPolar* histogram_polar,
                                                 double target_direction,
                                                 double driving_direction) {

  // The driving and goal directions are given in radians -> Convert to degrees
  target_direction = (int)floor((target_direction*180)/PI);
  if(driving_direction > PI || driving_direction < -PI){
    driving_direction += 360;
  }
  driving_direction = (int)floor((driving_direction*180)/PI);

  // Divide it by alpha because we want to find the best sector
  target_direction = (int)(floor(target_direction / histogram_polar->alpha));
  driving_direction = (int)(floor(driving_direction / histogram_polar->alpha));

  // Find our the direction of the target relative to our current orientation
  int relative_target_direction = 0;
  if(target_direction < driving_direction){
    relative_target_direction = 18 + abs(target_direction - driving_direction);
  } else {
    relative_target_direction = 18 - abs(target_direction - driving_direction);
  }
  // If relative_target_direction is less than 0 or greater than sectors,
  // we should spin
  // Create a vector to hold all candidate_valleys
  std::vector<std::pair<int, int>> candidate_valleys;

  // Loop through all sectors in the polar histogram to find valleys
  for (int sector = 0; sector < histogram_polar->sectors; ++sector) {
    if(histogram_polar->magnitudes[sector] < this->threshold){
      int valley_size = expand_valley(histogram_polar, sector);
      if(valley_size > this->robot_size*2){
        // Save the sector that starts the valley and the size of the valley
        candidate_valleys.push_back(std::make_pair(sector, valley_size));
      }
    }
  }
  int best_sector = 0;
  best_sector = find_best_sector(candidate_valleys,
                                 relative_target_direction,
                                 histogram_polar->sectors);
  // If it's -1, we did not find any better candidate sectors.
  // If it aims outside of our available sectors, something is weird
  if(best_sector == -1 || abs(best_sector) > histogram_polar->sectors){
    best_sector = relative_target_direction;
  }
  double new_direction = 0.0;
  if(best_sector < histogram_polar->origo){
    new_direction = -PI/2 + ((best_sector*histogram_polar->alpha*PI)/180);
  } else if(best_sector > histogram_polar->origo){
    new_direction = ((best_sector*histogram_polar->alpha*PI)/180) - PI/2;
  } else {
    new_direction = 0.0;
  }
  /*ROS_INFO_STREAM("Best sector is " << best_sector << " --> Turn " <<
                   new_direction << " radians.");*/
  this->best_sector = best_sector;
  return new_direction;
}

double VectorFieldHistogram::calculate_speed(HistogramPolar* histogram_polar){
  if(this->best_sector >= 0 && this->best_sector < histogram_polar->sectors){
    // magnitude > 0 indicates there is an obstacle
    if(histogram_polar->magnitudes[this->best_sector] > 0){
      double h_c = histogram_polar->magnitudes[this->best_sector];
      double h_c_prime = std::min(this->h_m, h_c);
      return (this->v_max * (1 - (h_c_prime / this->h_m)));
    } else{
      return this->v_max;
    }
  }
}
