#include "robot.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <cmath>
#include <chrono> // For std::chrono::seconds, std::chrono::milliseconds, etc.
#include <thread> // For std::this_thread::sleep_for
#include <cstdio> // Required for remove()
#include <iomanip> // For std::quoted (C++17 and later)
#include <system_error> // For std::error_code

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

    std::vector<float> dummyPoint = { 1.5,4 };
    std::cout << "Collision?: " << isCollosionFree(dummyPoint, obtacles) << "\n";

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
    direction.push_back( (point2.at(0) - point1.at(0)) / getDistance(point2, point1) );
    direction.push_back( (point2.at(1) - point1.at(1)) / getDistance(point2, point1) );
    return direction;

}

double robot::getAngle(const std::vector<float>& point2, const std::vector<float>& point1) {
    double pi = 2 * acos(0.0);
    double sinRatio = (point2.at(1) - point1.at(1)) / (getDistance(point2, point1));
    double cosRatio = (point2.at(0) - point1.at(0)) / (getDistance(point2, point1));
    double sin_angle = ( asin(sinRatio) );
    double cos_angle = ( acos(cosRatio) );
    double degrees;
    if (sinRatio >= 0.0) {
        //angle in 1st or 2nd quadrant
        if (cosRatio >= 0.0) {
            std::cout << "Quadrant 1: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 * sin_angle / pi;
        }
        else {
            std::cout << "Quadrant 2: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 * cos_angle / pi;
        }
    }
    else {
        //angle in 3rd or 4th quadrant
        if (cosRatio >= 0.0) {
            std::cout << "Quadrant 4: " << sin_angle << " " << cos_angle << "\n";
            degrees = 360.0 - 180.0 * cos_angle / pi;
        }
        else {
            std::cout << "Quadrant 3: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 - 180.0 * sin_angle / pi;
        }
    }
    return degrees;
}

float robot::getDistance(const std::vector<float>& point1, const std::vector<float>& point2) {
    return (std::sqrt(std::pow((point2.at(0) - point1.at(0)), 2) + std::pow((point2.at(1) - point1.at(1)), 2)) );
}

double robot::getAngleLines(const std::vector<float>& point0, const std::vector<float>& point1, const std::vector<float>& point2) {
    return (robot::getAngle(point2, point0) - robot::getAngle(point1, point0));
}

bool robot::isCollosionFree(const std::vector<float>& point, const std::vector< std::vector< std::vector<float> > >& obtacles) {
    for (size_t obstacleNumber = 0; obstacleNumber < obtacles.size(); ++obstacleNumber) {
        for (size_t vertexNumber = 0; vertexNumber < obtacles.at(obstacleNumber).size(); ++vertexNumber) {
            if (vertexNumber == obtacles.at(obstacleNumber).size() - 1) {//last vertex
                if (getAngleLines(point, obtacles.at(obstacleNumber).at(vertexNumber), obtacles.at(obstacleNumber).at(0)) >= 180.1)
                    break;
                return false;// if last vertex complete, but all below 180 (never break triggered), then point inside obstacle
            }
            else {//not last vertex
                if (getAngleLines(point, obtacles.at(obstacleNumber).at(vertexNumber), obtacles.at(obstacleNumber).at(vertexNumber + 1)) >= 180.1) {
                    break;//point outside this obstacle, no need to check for this obstacle anymore
                }
            }
        }
    }
    return true;
}

void robot::printPath(const std::vector< std::vector<float> >& path) {
    std::cout << "Displayed is the path of the robot\n\n";
    for (int pointNumber = 0; pointNumber < path.size(); pointNumber++) {
        std::cout << "Point " << (pointNumber+1) << " X: " << path.at(pointNumber).at(0) << " Y: " << path.at(pointNumber).at(1) << "\n";
    }
}

int robot::publishPath(const std::vector< std::vector<float> >& path,const std::string& csvFile) {
    std::ofstream outfile;

    std::ifstream f(csvFile);
    if (f.good()) { // File exists
        f.close();
        int result = std::remove(csvFile.c_str());
        if (result == 0) {
            std::cout << "Existing CSV file removed: " << csvFile << std::endl;
        }
        else {
            std::error_code ec(errno, std::generic_category());
            std::cerr << "Error removing existing CSV file: " << ec.message() << std::endl;
            return 1; // Or handle the error as needed
        }
    }
    else {
        std::cout << "Starting a new CSV file: " << csvFile << std::endl;
    }

    // Open file for writing (append mode is not used here, since we delete it)
    outfile.open(csvFile);

    if (!outfile.is_open()) {
        std::cerr << "Error opening CSV file for writing." << std::endl;
        return 1;
    }

    // Open the file in write mode
    std::ofstream file(csvFile);

    if (!file.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return 0;
    }

    // Iterate over each row of the 2D vector and write it to the CSV file
    for (const auto& row : path) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1) {
                file << ",";  // Add comma between values in the same row
            }
        }
        file << std::endl;  // Newline after each row
    }

    // Close the file
    file.close();
    std::cout << "File saved as " << csvFile << std::endl;

    return 1;
}