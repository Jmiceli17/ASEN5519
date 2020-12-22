#ifndef SHAPE_HPP_
#define SHAPE_HPP_

#include <iostream>
#include <vector>
using namespace std;

class Shape {

    public:
        virtual void DefineBoundary(double x_left, double x_right, double y_bottom, double y_top);

        virtual bool InCollision(std::vector<double> pt_to_check);
        //virtual void ApplyBoundaryOffset(double offset);

        virtual ~Shape(){};

};


#endif
