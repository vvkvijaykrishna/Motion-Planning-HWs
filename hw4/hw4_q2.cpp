#include "hw4_q2.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>

hw4_q2::hw4_q2(const std::string& inputFile) {
	readInputFile(inputFile);
}

void hw4_q2::readInputFile(const std::string& inputFile) {
	//description: This function is used to read the input csv file and obtain the values of link lengths and angles

	std::ifstream file(inputFile);
	if (!file.is_open()) {
		std::cerr << "The input file cannot be opened. Check if the file named inputFile_hw4.csv is present in the active folder and try again!\n";
	}

	std::string line;
	for (int lineNumber = 1; std::getline(file, line); ++lineNumber) {
		std::stringstream ss(line);
		std::string cell;
		for (int cellNumber = 1; std::getline(ss, cell, ','); ++cellNumber) {
			if ((lineNumber == 6 || lineNumber == 7 || lineNumber == 8) && cellNumber == 2)
				linkLengths.push_back(std::stod(cell));

			if ((lineNumber == 6 || lineNumber == 7 || lineNumber == 8) && cellNumber == 4)
			linkAngles.push_back(std::stod(cell));
		}
	}

}

std::vector <double> hw4_q2::endPointPosition() {
	//description: This function is used output the end-effectors position

	//x= cos(theta1)*l1 + cos(theta1+theta2)*l2 + cos(theta1+theta2+theta3)*l3
	endEffectorPosition.push_back(std::cos(linkAngles.at(0)) * linkLengths.at(0) + 
		std::cos(linkAngles.at(0) + linkAngles.at(1)) * linkLengths.at(1) +
		std::cos(linkAngles.at(0) + linkAngles.at(1) + linkAngles.at(2)) * linkLengths.at(2));

	//y= sin(theta1)*l1 + sin(theta1+theta2)*l2 + sin(theta1+theta2+theta3)*l3
	endEffectorPosition.push_back(std::sin(linkAngles.at(0)) * linkLengths.at(0) +
		std::sin(linkAngles.at(0) + linkAngles.at(1)) * linkLengths.at(1) +
		std::sin(linkAngles.at(0) + linkAngles.at(1) + linkAngles.at(2)) * linkLengths.at(2));

    std::cout << "End effector position: " << endEffectorPosition.at(0) << " , " << endEffectorPosition.at(1) << "\n\n";
	return endEffectorPosition;
}