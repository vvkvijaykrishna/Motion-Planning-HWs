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
    Node* robotNode = robot.getFromDef("PointRobot");
    Field* translationField = robotNode->getField("translation");

    for (const auto& point : path) {
        double targetX = point.at(0);
        double targetY = point.at(1);

        // Set the new position
        double newPosition[3] = { targetX, targetY, 0 };  // Keep Z = 0
        translationField->setSFVec3f(newPosition);

        // Wait before moving to the next point (simulating discrete movement)
        robot.step(1000);  // 1000 ms = 1 second
    }

    return 0;
}