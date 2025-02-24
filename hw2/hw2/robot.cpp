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
	//std::cout << "The inputFile to be used is " << inputFile << std::endl;

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
    //std::cout << "All input data is extracted from chosen .csv file\n";

    //std::vector<float> dummyPoint = { 12.1,12 };
    //std::cout << "Collision?: " << isCollosionFree(dummyPoint, obtacles) << "\n";

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
    std::vector<float> next;
    std::vector<float> hitPoint; // record the thit point too!
    bool aroundObstacle = false; // to determine whether the robot is circulating an obstacle or not
    bool isLeft = true; //true for left, false for right
    double pi = 2 * acos(0.0); //pi value

    current = start;
    next = current;
    
    path.push_back(current);

    //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Done till here!\n";

    while (getDistance(current, goal) > epsilon) {
        std::vector<float> leastDistantPoint = { 100.0,100.0 };
        while (getDistance(current, goal) > epsilon && aroundObstacle == false) {// a while loop here to move towards goal
            direction = getDirection(goal, current);
            next.at(0) = current.at(0) + step * direction.at(0);
            next.at(1) = current.at(1) + step * direction.at(1);
            if (!isCollosionFree(next, obtacles)) {
                hitPoint = current; // recording the hit point here
                aroundObstacle = true;
                //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Hit point found!\n";
            }
            else {
                current.at(0) = next.at(0);
                current.at(1) = next.at(1);
                //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Following goal\n";
                path.push_back(current);
            }
        }

        //once it hits an obstacle, should record the hit point, and start going around the obstacle
        while (aroundObstacle == true && getDistance(current, goal) > epsilon) {
            //direction is to left of the point just interfereing with the obstacle, find the direction
            bool direction_found = false;
            std::vector<float> directionCW;
            std::vector<float> directionCCW;
            int directionLoopCount = 0;

            while (!direction_found) {// while loop to find the direction
                // if current dir is collision free, if dirCW is collision free, then dir = dirCW, else, return dir
                // else, dir = dirCCW
                //std::cout << "\nIn direction_found loop\n";
                next.at(0) = current.at(0) + step * direction.at(0);
                next.at(1) = current.at(1) + step * direction.at(1);
                if (isCollosionFree(next, obtacles)) {
                    directionCW = rotateAngle(direction, -30.0);
                    next.at(0) = current.at(0) + step * directionCW.at(0);
                    next.at(1) = current.at(1) + step * directionCW.at(1);
                    if (isCollosionFree(next, obtacles)) {
                        direction = directionCW;
                    }
                    else {
                        direction_found = true;
                        break;
                    }
                }
                else {
                    directionCCW = rotateAngle(direction, 30.0);
                    direction = directionCCW;
                }
                if (directionLoopCount++ > 20) {
                    std::cout << "Trying to find the direction since 20 attempts\nExiting the program\n";
                    exit(0);
                }

            }

            next.at(0) = current.at(0) + step * direction.at(0);
            next.at(1) = current.at(1) + step * direction.at(1);
            current.at(0) = next.at(0);
            current.at(1) = next.at(1);
            current.push_back(getDistance(current, goal));
            path.push_back(current);
            //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Following obstacle!\n";
            //std::cout << "getDistance(current, goal) " << getDistance(current, goal) << " getDistance(leastDistantPoint, goal) " << getDistance(leastDistantPoint, goal) << " \n";
            if ( getDistance(current, goal) < getDistance(leastDistantPoint, goal) ) {
                leastDistantPoint = current;
                //std::cout << leastDistantPoint.at(0) << " " << leastDistantPoint.at(1) << " " << getDistance(leastDistantPoint, goal) << " Least Distant Point changed!\n";
            }

            if (getDistance(current, hitPoint) < epsilon) {
                std::vector< std::vector<float> > revolveObstacle; // path of robot following the obstacle
                //std::cout << leastDistantPoint.at(0) << " " << leastDistantPoint.at(1) << " " << getDistance(leastDistantPoint, goal) << " Least Distant Point!\n";
                //std::cout << current.at(0) << " " << current.at(1) << " " << getDistance(current, goal) << " Reverse obstacle!\n";

                auto iterationHitPoint = std::find(path.begin(), path.end(), hitPoint);
                auto iterationLeastDistantPoint = std::find(path.begin(), path.end(), leastDistantPoint);

                int indexHitPoint = std::distance(path.begin(), iterationHitPoint);
                int indexLeastDistantPoint = std::distance(path.begin(), iterationLeastDistantPoint);

                if ( (path.size() - indexLeastDistantPoint) <= (indexLeastDistantPoint - indexHitPoint) ) {// then go CCW, else fo CW
                    for (int i = (path.size() - 2); path.at(i) != leastDistantPoint; --i) {
                        revolveObstacle.push_back(path.at(i));
                    }
                }
                else {
                    for (int i = indexHitPoint; path.at(i) != leastDistantPoint; ++i) {
                        revolveObstacle.push_back(path.at(i));
                    }
                }

                for (int i = 0; i < revolveObstacle.size(); i++) {
                    path.push_back(revolveObstacle.at(i));
                    //std::cout << revolveObstacle.at(i).at(0) << " " << revolveObstacle.at(i).at(1) << " Reverse obstacle!\n";
                }
                aroundObstacle = false;
                current = leastDistantPoint;
            }
        }
    }
    return path;
}

std::vector< std::vector<float> > robot::bug2(const float& step, const float& epsilon) {
    std::vector< std::vector<float> > path;
    std::vector<float> direction;
    std::vector<float> current;
    std::vector<float> next;
    std::vector<float> mLine = { (goal.at(1) - start.at(1)), (start.at(0) - goal.at(0)), ( start.at(0)*(start.at(1) - goal.at(1)) + start.at(1)*(goal.at(0) - start.at(0)) ) };// obtaining the m-line here
    bool aroundObstacle = false; // to determine whether the robot is circulating an obstacle or not
    bool isLeft = true; //true for left, false for right
    double pi = 2 * acos(0.0); //pi value

    current = start;
    next = current;

    path.push_back(current);

    while (getDistance(current, goal) > epsilon) {
        //bool obstacleFirstTime = true;
        while (getDistance(current, goal) > epsilon && aroundObstacle == false) {// a while loop here to move towards goal
            direction = getDirection(goal, current);
            next.at(0) = current.at(0) + step * direction.at(0);
            next.at(1) = current.at(1) + step * direction.at(1);
            if (!isCollosionFree(next, obtacles)) {
                aroundObstacle = true;
                //std::cout << current.at(0) << " " << current.at(1) << " " << " Hit point found!\n";
            }
            else {
                current.at(0) = next.at(0);
                current.at(1) = next.at(1);
                //std::cout << current.at(0) << " " << current.at(1) << " " << " Following goal\n";
                path.push_back(current);
            }
        }
        //once it hits an obstacle, should record the hit point, and start going around the obstacle
        while (aroundObstacle == true && getDistance(current, goal) > epsilon) {
            //direction is to left of the point just interfereing with the obstacle, find the direction
            bool direction_found = false;
            std::vector<float> directionCW;
            std::vector<float> directionCCW;
            int directionLoopCount = 0;

            while (!direction_found) {// while loop to find the direction
                // if current dir is collision free, if dirCW is collision free, then dir = dirCW, else, return dir
                // else, dir = dirCCW
                //std::cout << "\nIn direction_found loop\n";
                next.at(0) = current.at(0) + step * direction.at(0);
                next.at(1) = current.at(1) + step * direction.at(1);
                if (isCollosionFree(next, obtacles)) {
                    directionCW = rotateAngle(direction, -30.0);
                    next.at(0) = current.at(0) + step * directionCW.at(0);
                    next.at(1) = current.at(1) + step * directionCW.at(1);
                    if (isCollosionFree(next, obtacles)) {
                        direction = directionCW;
                    }
                    else {
                        direction_found = true;
                        break;
                    }
                }
                else {
                    directionCCW = rotateAngle(direction, 30.0);
                    direction = directionCCW;
                }
                if (directionLoopCount++ > 20) {
                    std::cout << "Trying to find the direction since 20 attempts\nExiting the program\n";
                    exit(0);
                }

            }

            next.at(0) = current.at(0) + step * direction.at(0);
            next.at(1) = current.at(1) + step * direction.at(1);
            
            current.at(0) = next.at(0);
            current.at(1) = next.at(1);
            current.push_back(getDistance(current, goal));
            path.push_back(current);
            //std::cout << current.at(0) << " " << current.at(1) << " " << " Following obstacle!\n";
            //std::cout << "getDistance(current, goal) " << getDistance(current, goal) << " getDistance(leastDistantPoint, goal) " << getDistance(leastDistantPoint, goal) << " \n";

            //check the condition of m-line, if true, then change aroundObstacle, else let it pass
            //std::cout << mLine.at(0) << " " << mLine.at(1) << " " << mLine.at(2) << " Mline equation!\n";
            //std::cout << "The distance to Mline is: " << distanceToMline(mLine, current) << "\n\n";
            if (distanceToMline(mLine, current) < epsilon) {
                aroundObstacle = false;
                //std::cout << current.at(0) << " " << current.at(1) << " " << " M-Line found!!!\n";
            }
        }
    }
    return path;
}

float robot::distanceToMline(const std::vector<float>& mLine, const std::vector<float>& point) {
    return ( std::abs(mLine.at(0) * point.at(0) + mLine.at(1) * point.at(1) + mLine.at(2)) / std::sqrt(mLine.at(0) * mLine.at(0) + mLine.at(1) * mLine.at(1)) );
}

std::vector<float> robot::rotateAngle(const std::vector<float>&currentDirection, const double& diffAngle) {
    std::vector<float> zero_point = { 0.0,0.0 };
    std::vector<float> newDirection = { 0.0,0.0 };
    double pi = 2 * acos(0.0);
    double current_angle = getAngle(currentDirection, zero_point);
    double new_angle = current_angle + diffAngle;
    //std::cout << "\nCurrent angle " << current_angle << ", New angle " << new_angle << "\n";
    if (new_angle > 360.0) {
        new_angle -= 360.0;
    }
    newDirection.at(0) = cos(new_angle * pi / 180);
    newDirection.at(1) = sin(new_angle * pi / 180);
    return newDirection;
}


std::vector<float> robot::getDirection(const std::vector<float>& point2, const std::vector<float>& point1) {
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
            //std::cout << "Quadrant 1: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 * sin_angle / pi;
        }
        else {
            //std::cout << "Quadrant 2: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 * cos_angle / pi;
        }
    }
    else {
        //angle in 3rd or 4th quadrant
        if (cosRatio >= 0.0) {
            //::cout << "Quadrant 4: " << sin_angle << " " << cos_angle << "\n";
            degrees = 360.0 - 180.0 * cos_angle / pi;
        }
        else {
            //std::cout << "Quadrant 3: " << sin_angle << " " << cos_angle << "\n";
            degrees = 180.0 - 180.0 * sin_angle / pi;
        }
    }
    return degrees;
}

float robot::getDistance(const std::vector<float>& point1, const std::vector<float>& point2) {
    return (std::sqrt(std::pow((point2.at(0) - point1.at(0)), 2) + std::pow((point2.at(1) - point1.at(1)), 2)) );
}

double robot::getAngleLines(const std::vector<float>& point0, const std::vector<float>& point1, const std::vector<float>& point2) {
    double angle = (robot::getAngle(point2, point0) - robot::getAngle(point1, point0));
    if (angle < 0.0) {
        angle += 360.0;
    }
    return angle;
}

bool robot::isCollosionFree(const std::vector<float>& point, const std::vector< std::vector< std::vector<float> > >& obtacles) {
    for (size_t obstacleNumber = 0; obstacleNumber < obtacles.size(); ++obstacleNumber) {
        for (size_t vertexNumber = 0; vertexNumber < obtacles.at(obstacleNumber).size(); ++vertexNumber) {
            if (vertexNumber == obtacles.at(obstacleNumber).size() - 1) {//last vertex
                double angle = getAngleLines(point, obtacles.at(obstacleNumber).at(vertexNumber), obtacles.at(obstacleNumber).at(0));
                //std::cout << angle << " is the angle between the given point and obstacleNumber " << obstacleNumber << " at vertex " << vertexNumber << ", 0\n";
                if (angle >= 180.1)
                    break;
                //std::cout << "Point inside obstacle!!\n";
                return false;// if last vertex complete, but all angles below 180 (never break triggered), then point inside obstacle
            }
            else {//not last vertex
                double angle = getAngleLines(point, obtacles.at(obstacleNumber).at(vertexNumber), obtacles.at(obstacleNumber).at(vertexNumber + 1));
                //std::cout << angle << " is the angle between the given point and obstacleNumber " << obstacleNumber << " at vertex " << vertexNumber << ", " << (vertexNumber + 1)<<"\n";
                if (angle >= 180.1) {
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
            //std::cout << "Existing CSV file removed: " << csvFile << std::endl;
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

    outfile.open(csvFile);
    if (!outfile.is_open()) {
        std::cerr << "Error opening CSV file for writing." << std::endl;
        return 1;
    }

    std::ofstream file(csvFile);
    if (!file.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return 0;
    }

    for (const auto& row : path) {
        file << row[0] << "," << row[1] << std::endl;
    }

    file.close();
    //std::cout << "File saved as " << csvFile << std::endl;

    return 1;
}