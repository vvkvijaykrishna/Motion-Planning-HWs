#pragma once
#include <string>
#include <vector>

class robot
{
public:
	robot(std::string inputFile = "inputFile.csv");

	void readInputFile(const std::string& inputFile);
	std::vector<float> processCSVcell(const std::string& cell);
	std::vector< std::vector<float> > bug1(const float& step, const float& epsilon);
	std::vector<float> rotateAngle(const std::vector<float>& currentDirection, const double& diffAngle);
	std::vector<float> getDirection(std::vector<float>& point1, std::vector<float>& point2);
	float getDistance(const std::vector<float>& point1, const std::vector<float>& point2);
	double getAngleLines(const std::vector<float>& point0, const std::vector<float>& point1, const std::vector<float>& point2);
	double getAngle(const std::vector<float>& point1, const std::vector<float>& point2);
	bool isCollosionFree(const std::vector<float>& point, const std::vector< std::vector< std::vector<float> > >& obtacles);
	void printPath(const std::vector< std::vector<float> >& path);
	int publishPath(const std::vector< std::vector<float> >& path, const std::string& csvFile = "robotPath.csv");

private:
	std::vector< std::vector<float> > workspaceLimits;
	std::vector<float> start;
	std::vector<float> goal;
	std::vector< std::vector< std::vector<float> > > obtacles;
};