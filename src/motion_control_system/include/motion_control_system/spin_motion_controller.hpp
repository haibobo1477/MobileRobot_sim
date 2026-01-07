#ifndef SPIN_MOTION_CONTROL_HPP
#define SPIN_MOTION_CONTROL_HPP

#include "motion_control_system/motion_control_interface.hpp"


namespace motion_control_system
{
    class SpinMotionController : public MotionController
    {
    private:
    public:
        virtual void start() override;
        virtual void stop() override;
    };
}



#endif // SPIN_MOTION_CONTROL_HPP