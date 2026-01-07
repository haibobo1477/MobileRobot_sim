#include "motion_control_system/motion_control_interface.hpp"
#include <pluginlib/class_loader.hpp>


int main(int argc, char **argv)
{
    
    if(argc != 2)
       return 0;

    std::string controller_name = argv[1];
    pluginlib::ClassLoader<motion_control_system::MotionController> controller_loader("motion_control_system",
                                                                                      "motion_control_system::MotionController");

    auto controller = controller_loader.createSharedInstance(controller_name);
    
    controller->start();
    controller->stop();
    
    return 0;
}