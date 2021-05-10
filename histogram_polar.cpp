#include "air_lab2/histogram_polar.h"

HistogramPolar::HistogramPolar(){
  this->alpha = 5;

}

HistogramPolar::HistogramPolar(int _alpha) {
  // alpha = 5 in the paper
  this->alpha = _alpha;
  // 180 degree vision with laserscan
  this->sectors = int(floor(180/this->alpha));
  this->origo = this->sectors/2;
  this->magnitudes = std::vector<int>(this->sectors, 0);
}

void HistogramPolar::update_polar(HistogramGrid* grid, double current_x,
                                  double current_y){
  int w_s = grid->getDimension();
  /* a - (b * dmax) = 0, where dmax = sqrt(2)* ((ws-1)/2) */
  double d_max = (sqrt(2)*(w_s-1))/2;
  // Divide with grid resolution to get distance as number of cells.
  //double distance = d_ij / grid->getResolution();

  double b = 5.0; // arbitrary number, needs to be evaluated
  double a = b*d_max;
  double beta = 0.0;
  for(int x = 0; x < w_s; x++){
    for(int y = 0; y < (w_s/2); y++){
      beta = atan2(y, x);
      int dist_x = abs(x - w_s/2);
      int dist_y = abs(y - w_s/2);
      double d_ij = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
      d_ij /= grid->getResolution();
      beta = (int)((beta*180)/PI);
      int certainty_value = pow(grid->getCell(x,y), 2);
      double magnitude = certainty_value*(a-b*d_ij);
      int k = (int)(beta / this->alpha);
      this->magnitudes[k] = magnitude;
    }
  }
  for(int x = 0; x < w_s; x++){
    for(int y = (w_s/2); y < w_s; y++){
      int dist_x = abs(x - w_s/2);
      int dist_y = abs(y - w_s/2);
      double d_ij = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
      d_ij /= grid->getResolution();
      beta = atan2(y, x);
      beta = (int)((beta*180)/PI);
      int certainty_value = pow(grid->getCell(x,y), 2);
      double magnitude = certainty_value*(a-b*d_ij);
      int k = (int)(beta / this->alpha);
      k += this->origo;
      this->magnitudes[k] = magnitude;
    }
  }
}

double HistogramPolar::getSmoothedValue(int k, int l){
  double hk_prime = 0.0;
  for(int i = k-(l+1); i < k+l; i++){
    if(i <= k){
      hk_prime += (i-k+l)*this->magnitudes[i];
    } else {
      hk_prime += (k-i+l)*this->magnitudes[i];
    }
  }
  return hk_prime/((2*l)-1);
}
// l = 5 is good according to article
void HistogramPolar::smooth_histogram(int l){
  for(int k = l; k < this->sectors+1; k++){
    this->magnitudes[k] = getSmoothedValue(k, l);
  }
}
