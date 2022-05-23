#include "fake_orientation.hpp"
#include <cmath>
#include <vector>
#include <iostream>

namespace fix_modes
{
enum FIX_MODE
{
  INVALID = 0,
  SINGLE_POINT_POSITION,
  DIFFERENTIAL_GNSS,
  FLOAT_RTK,
  FIXED_RTK,
  DEAD_RECKONING,
  SBAS_POSITION
};
}

void FakeOri::addXY(double x, double y) {
    if(x_vect.size() >= 10){
        x_vect.erase(x_vect.begin());
        y_vect.erase(y_vect.begin());
    }
    x_vect.push_back(x);
    y_vect.push_back(y);
}

void FakeOri::setStatus(int fix_mode) {
    status = fix_mode;
}

void FakeOri::printAll() {
    for (int i = 0; i < x_vect.size(); i++) {
        std::cout << x_vect[i] - x_vect[0] << " " << y_vect[i] - y_vect[0] << ", ";
    }
    std::cout << "\n";
    std::cout << "\n --" << x_vect.size() << "-- ";
}

double FakeOri::calcOrientation(int p1, int p2) const {
    return atan2(y_vect[p1] - y_vect[p2], x_vect[p1] - x_vect[p2]);
}

double FakeOri::getDistance() const {
    double dist = abs(sqrt(pow(y_vect[0] - y_vect[9], 2) + pow(x_vect[0] - x_vect[9], 2)));
    //std::cout << "\n **" << dist << "** ";
    return dist;
}

double FakeOri::getOri() const {
    double ret = fake_orientation;
    // if there is not enough mesaurement
    if (x_vect.size() < 10 ){
        return ret;
    }
    if(getDistance() < 0.1){
        return ret;
        //std::cout << "aaa\n";
    } 
    if(status == fix_modes::FIXED_RTK or status == fix_modes::FLOAT_RTK){
        ret = calcOrientation(0, 4);
    }
    else if(status == fix_modes::INVALID){
        return ret;
    }
    else{
        ret = calcOrientation(0, 9);
    }
    return ret;
}




