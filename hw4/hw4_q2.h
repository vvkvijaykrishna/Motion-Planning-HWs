#include <string>
#include <vector>

class hw4_q2
{
public:
	hw4_q2(const std::string& inputFile);

	void readInputFile(const std::string& inputFile);
	std::vector <double> endPointPosition();

private:
	std::vector <double> linkLengths;
	std::vector <double> linkAngles;
	std::vector <double> endEffectorPosition;
};

