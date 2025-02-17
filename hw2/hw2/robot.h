#pragma once
#include <string>
#include <vector>

class robot
{
public:
	robot(std::string inputFile = "inputFile.csv");

	void readInputFile(const std::string& inputFile);
	std::vector<int> processCSVcell(const std::string& cell);
	std::vector< std::vector<int> > bug1();
	//std::vector< std::vector<int> > bug2();

private:
	std::vector< std::vector<int> > workspaceLimits;
	std::vector<int> start;
	std::vector<int> goal;
	std::vector< std::vector< std::vector<int> > > obtacles;
	std::vector< std::vector<int> > path;
};