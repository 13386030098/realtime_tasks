#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris.hpp>
#include <atomic>
#include <queue>

//Function declaration
using Size = std::size_t;
constexpr double PI = 3.141592653589793;
#define TEST_SIZE 2048
std::queue<double> joint_1;

class SetVel : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
    explicit SetVel(const std::string &name = "SetVel_plan");
    ARIS_REGISTER_TYPE(SetVel)
};

class Motion : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
    explicit Motion(const std::string &name = "Motion_plan");
    ARIS_REGISTER_TYPE(Motion)
};



#endif
