#include "robot.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>

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
        std::vector< std::vector<int> > singleObstacle;
        for (int cellNumber = 1; (std::getline(ss, cell, '"')); cellNumber++) {
            std::cout << cell << cellNumber << " ";
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
        std::cout << lineNumber << std::endl;
    }
    //std::cout << "Obstacle coordinates: " << obtacles.at(0).at(3).at(0) << obtacles.at(1).at(3).at(0) << obtacles.at(2).at(3).at(0) << obtacles.at(3).at(3).at(0) << obtacles.at(4).at(3).at(0);
    std::cout << "All input data is extracted from chosen .csv file\n";
    file.close();
}

std::vector<int> robot::processCSVcell(const std::string& cell) {
    std::regex pattern(R"(\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\))");// Regex to extract (+-x, +-y)
    std::smatch match;
    std::vector<int> num;

    if (std::regex_match(cell, match, pattern)) {
        int firstNum = std::stoi(match[1]);  // Extract first number
        int secondNum = std::stoi(match[2]); // Extract second number

        std::cout << "Extracted Numbers: " << firstNum << ", " << secondNum << std::endl;
        num.push_back(firstNum);
        num.push_back(secondNum);
    }
    else {
        std::cout << "Match not possible: " << cell << std::endl; // Print normally if not in (x,y) format
    }

    return num;
}

std::vector< std::vector<int> > bug1() {

}


