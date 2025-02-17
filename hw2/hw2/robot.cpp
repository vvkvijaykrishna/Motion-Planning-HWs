#include "robot.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <cmath>
#include <chrono>   // For std::chrono::seconds, std::chrono::milliseconds, etc.
#include <thread>   // For std::this_thread::sleep_for

robot::robot(std::string inputFile) {
	readInputFile(inputFile); //this command is used to obtain the input parameters (start, goal, obstacles)
}

void robot::readInputFile(const std::string& inputFile)
{
	std::cout << "The inputFile to be used is " << inputFile << std::endl;

	std::ifstream file(inputFile);
	if (!file.is_open()) {
		std::cerr << "Could not open the file!" << std::endl;
	}

    std::string line;

    for (int lineNumber = 1; std::getline(file, line); lineNumber++) {
        std::stringstream ss(line);
        std::string cell;
        std::vector< std::vector<float> > singleObstacle;
        for (int cellNumber = 1; (std::getline(ss, cell, '"')); cellNumber++) {
            //std::cout << cell << cellNumber << " ";
            if (lineNumber == 6 && cellNumber == 2)
                workspaceLimits.push_back(processCSVcell(cell));
            if (lineNumber == 7 && cellNumber == 2)
                workspaceLimits.push_back(processCSVcell(cell));
            if (lineNumber == 9 && cellNumber == 2)
                start = processCSVcell(cell);
            if (lineNumber == 10 && cellNumber == 2)
                goal = processCSVcell(cell);
            if (lineNumber >= 13) {
                if (cellNumber % 2 == 0) {
                    singleObstacle.push_back(processCSVcell(cell));
                }
            }
        }
        if (!singleObstacle.empty()) {
            obtacles.push_back(singleObstacle);
        }
        //std::cout << lineNumber << std::endl;
    }
    //std::cout << "Obstacle coordinates: " << obtacles.at(0).at(3).at(0) << obtacles.at(1).at(3).at(0) << obtacles.at(2).at(3).at(0) << obtacles.at(3).at(3).at(0) << obtacles.at(4).at(3).at(0);
    std::cout << "All input data is extracted from chosen .csv file\n";
    file.close();
}

std::vector<float> robot::processCSVcell(const std::string& cell) {
    //std::regex pattern(R"(\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\))");// Regex to extract (+-x, +-y)
    std::regex pattern(R"(\(\s*(-?\d+\.\d+|-?\d+)\s*,\s*(-?\d+\.\d+|-?\d+)\s*\))");
    std::smatch match;
    std::vector<float> num;

    if (std::regex_match(cell, match, pattern)) {
        float firstNum = float(std::stoi(match[1]));  // Extract first number
        float secondNum = float(std::stoi(match[2])); // Extract second number

        std::cout << "Extracted Numbers: " << firstNum << ", " << secondNum << std::endl;
        num.push_back(firstNum);
        num.push_back(secondNum);
    }
    else {
        std::cout << "Match not possible: " << cell << std::endl; // Print normally if not in (x,y) format
    }

    return num;
}

std::vector< std::vector<float> > robot::bug1(const float& step, const float& epsilon) {
    std::vector< std::vector<float> > path;
    std::vector<float> direction;
    std::vector<float> current;

    current = start;
    path.push_back(current);

    //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Done tille here!\n";

    while (getDistance(current, goal) > epsilon) {
        direction = getDirection(goal, current);
        //std::cout << "Direction: " << direction.at(0) << " " << direction.at(1) << "\n";
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        current.at(0) += step * direction.at(0);
        current.at(1) += step * direction.at(1);
        path.push_back(current);
        //std::cout << "Step: " << step << "Direction: " << direction.at(0) << " " << direction.at(1);
        //std::cout << "Current: " << current.at(0) << " " << current.at(1) << "\n";

    }
    return path;
}

std::vector<float> robot::getDirection(std::vector<float>& point2, std::vector<float>& point1) {
    std::vector<float> direction;
    direction.push_back( (point2.at(0) - point1.at(0)) / getDistance(point1, point2) );
    direction.push_back( (point2.at(1) - point1.at(1)) / getDistance(point1, point2) );
    return direction;

}

float robot::getDistance(std::vector<float>& point1, std::vector<float>& point2) {
    return (std::sqrt(std::pow((point2.at(0) - point1.at(0)), 2) + std::pow((point2.at(1) - point1.at(1)), 2)) );
}

void robot::printPath(const std::vector< std::vector<float> >& path) {
    std::cout << "Displayed is the path of the robot\n\n";
    for (int pointNumber = 0; pointNumber < path.size(); pointNumber++) {
        std::cout << "Point " << (pointNumber+1) << " X: " << path.at(pointNumber).at(0) << " Y: " << path.at(pointNumber).at(1) << "\n";
    }
}


