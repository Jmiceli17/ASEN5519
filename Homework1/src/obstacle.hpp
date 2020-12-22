#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_
#include "shape.hpp"
#include <iostream>
#include <vector>
using namespace std;



class AlignedRectangle : public Shape {


	private:
    // Primitives
    bool P0(std::vector<double> pt_to_check);	// left vertical
    bool P1(std::vector<double> pt_to_check);
    bool P2(std::vector<double> pt_to_check);	// right vertical
    bool P3(std::vector<double> pt_to_check);


    public:
        double min_x;
        double max_x;
        double min_y;
        double max_y;
        std::vector<double> v4;
        std::vector<double> v3;
        std::vector<double> v2;
        std::vector<double> v1;
        bool obstacle_traced;
        std::vector<std::vector<double>> vertex_vector;



        AlignedRectangle();
        //AlignedRectangle(double x_left, double x_right, double y_bottom, double y_top);
        AlignedRectangle(std::vector<std::vector<double>> vertex_vector);


        ~AlignedRectangle();

        void DefineBoundary(double x_left, double x_right, double y_bottom, double y_top);

        // Define the primatives from the verticies of the obstacle
        //void Primiatives(std::vector<std::vector<double>> vertex_vector);

        // Check if a given point is in or on the obstacle
        bool InCollision(std::vector<double> pt_to_check);

        //void ApplyBoundaryOffset(double offset);



};


#endif
