#pragma comment(lib, "C:\\Program Files\\Webots\\lib\\controller\\Controller.lib")

#include "simulator.h"
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <vector>

#include <windows.h>

using namespace webots;

simulator::simulator() {
}

int simulator::simulatePath(const std::vector< std::vector<float> >& path) {

    Supervisor robot;
    Node* robotNode = robot.getFromDef("point_robot_controller");
    //Node* robotNode = robot.getFromDef("PointRobot");
    Field* translationField = robotNode->getField("translation");

	for (size_t i = 0; i < path.size() - 1; i++) {
		std::vector<float> current = path.at(i);
		std::vector<float> next = path.at(i+1);

        for (float alpha = 0.0; alpha <= 1.0; alpha += 0.1) {
            float intermediateX = current.at(0) * (1 - alpha) + next.at(0) * alpha;
            float intermediateY = current.at(1) * (1 - alpha) + next.at(1) * alpha;

            double newPosition[3] = { double(intermediateX), double(intermediateY), 0.5 };
            translationField->setSFVec3f(newPosition);

            robot.step(100);

        }
	}

    return 0;
}