#pragma once
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <vector>

#include <windows.h>

using namespace webots;

class simulator
{
public:
	simulator();
	int simulatePath(const std::vector< std::vector<float> >& path);

};