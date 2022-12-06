#ifndef FAKE_ORI_HPP
#define FAKE_ORI_HPP
#include <vector>
#include <iostream>


class FakeOri{
  public:  
    double fake_orientation;
    int status;
    std::vector <double> x_vect;
    std::vector <double> y_vect;
    std::vector <double> ori_vect;
    void addXY(double x, double y);
    void setStatus(int fix_mode);
    void printAll();
    double calcOrientation(int p1, int p2) const;
    double getDistance() const;
    double getOri() const;

};

#endif