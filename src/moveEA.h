#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris.hpp>
#include <atomic>

//Function declaration
using Size = std::size_t;
constexpr double PI = 3.141592653589793;

class SetVel : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit SetVel(const std::string &name = "SetVel_plan");
    ARIS_REGISTER_TYPE(SetVel);
};


#endif
