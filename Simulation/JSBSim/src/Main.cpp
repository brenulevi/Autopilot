#include <iostream>
#include "Autopilot/Control/Control.h"
#include "FGFDMExec.h"

int main(int argc, char** argv)
{
    std::cout << "Hello, JSBSim!" << std::endl;
    std::cout << "JSBSim version: " << JSBSim::FGFDMExec::GetVersion() << std::endl;

    Control_t control;
    Control_Init(&control);

    return 0;
}