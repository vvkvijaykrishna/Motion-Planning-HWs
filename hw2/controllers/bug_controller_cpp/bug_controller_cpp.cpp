#include <webots/Supervisor.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

using namespace webots;
using namespace std;

vector<vector<double>> getRobotPath();

int main(int argc, char **argv) {
    Supervisor *robot = new Supervisor();
    int timestep = (int)robot->getBasicTimeStep();
    Node *robot_node = robot->getSelf();
    Field *translation_field = robot_node->getField("translation");

    //vector<vector<double>> coordinates = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {6, 6}, {8, 8}};
    vector<vector<double>> coordinates = getRobotPath();
    int coordinate_index = 0;

    while (robot->step(timestep) != -1) {
        if (coordinate_index < coordinates.size()) {
            double target_x = coordinates[coordinate_index][0];
            double target_y = coordinates[coordinate_index][1];
            double target_z = 0.5;

            // Set the robot's position using the translation field
            double new_translation[3] = {target_x, target_y, target_z};
            translation_field->setSFVec3f(new_translation);

            //cout << "Moving to: (" << target_x << ", " << target_y << ")" << endl;

            // Introduce a 100ms delay
            int delay_steps = 3 / timestep;
            for (int i = 0; i < delay_steps; ++i) {
                robot->step(timestep);
            }

            coordinate_index++;
        } else {
            cout << "Path completed." << endl;
            break;
        }
    }

    delete robot;
    return 0;
}

vector<vector<double>> getRobotPath(){
  std::ifstream file("robotPath.csv");
  std::vector< std::vector<double> > path;
  if (!file.is_open()) {
    std::cerr << "Could not open the file!" << std::endl;
  }
  std::string line;

  for (int lineNumber = 1; std::getline(file, line); lineNumber++) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> coordinate;
    for (int cellNumber = 1; (std::getline(ss, cell, ',')); cellNumber++) {
            if (cellNumber == 1){
                coordinate.push_back(std::stod(cell));}
            if (cellNumber == 2){
                coordinate.push_back(std::stod(cell));}
    }
    path.push_back(coordinate);
  }
  return path;
}