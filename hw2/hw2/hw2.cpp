// hw2.cpp : This file contains the 'main' function. Trying to implement bug 1 algorithm here

#include <iostream>
#include "robot.h"
//#include "simulator.h"

int main()
{   
    float step = 0.1;
    float epsilon = 0.1;
    std::vector<float> point0 = { 0,0 };
    std::vector<float> point1 = { 1,1 };
    std::vector<float> point2 = { 0,1 };
    robot bug;
    //std::cout << "Angle is: " << bug.getAngleLines(point0, point1, point2);
    std::vector< std::vector<float> > path = bug.bug1(step, epsilon);
    bug.printPath(path);
    bug.publishPath(path);

    return 0;
    //write functionality to parse .csv input file - each obstacle should contain atleast 3 vertices
    //stitch obstacles intersecting each other

    //- class inputs
    //- class
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
