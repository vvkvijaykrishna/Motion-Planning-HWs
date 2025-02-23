// hw2.cpp : This file contains the 'main' function. Trying to implement bug 1 algorithm here

#include <iostream>
#include <chrono>
#include "robot.h"
//#include "simulator.h"

int main()
{   
    auto start = std::chrono::high_resolution_clock::now();
    float step = 0.1;
    float epsilon = 0.1;
    robot bug;
    std::vector< std::vector<float> > path = bug.bug1(step, epsilon);
    //bug.printPath(path);
    bug.publishPath(path);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Total execution time: " 
        << std::chrono::duration<double, std::milli>(end - start).count() << " ms\n";
  
    return 0;
    //Proablems/ concerns
    //1. Pubsish .csv file excluding the distance values too - done
    //2. Bug algorithm should take the least possible path to the obstacle exit point.
    //3. Monitor time required to run the script.
    //4. Create a python file which simulates the .csv output file
    //5. Write the Bug2 algorithm with the m-line (must be simple). Create a new function for this.

    //write functionality to parse .csv input file - each obstacle should contain atleast 3 vertices

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
