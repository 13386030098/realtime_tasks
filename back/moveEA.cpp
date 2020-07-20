#include <iostream>
#include <aris.hpp>
#include "moveEA.h"
#include <Eigen/Eigen>
#include <math.h>
#include <string>
#include <signal.h>
#include <queue>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace aris::plan;

#define pi M_PI
typedef void(*signal_handler);

//creat ethercat controller，upload xml message
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
    //4096 represent 4096 counts/revolution
    double pos_factor_roll = 4096.0*120/2/PI;
//    double pos_factor_link = 4096.0*100/2/PI;

    std::string xml_str_roll =
        "<EthercatMotion phy_id=\"0\" product_code=\"0x00030924\""
        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_roll)+"\" pos_offset=\"0.0\">"
        "	<SyncManagerPoolObject>"
        "		<SyncManager is_tx=\"false\"/>"
        "		<SyncManager is_tx=\"true\"/>"
        "		<SyncManager is_tx=\"false\">"
        "           <Pdo index=\"0x1605\" is_tx=\"false\">"
        "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
        "               <PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"max_tor\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        "               <PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        "           </Pdo>"
        "           <Pdo index=\"0x161D\" is_tx=\"false\">"
        "               <PdoEntry name=\"target_pos\" index=\"0x60FE\" subindex=\"0x01\" size=\"32\"/>"
        "           </Pdo>"
        "		</SyncManager>"
        "		<SyncManager is_tx=\"true\">"
        "			<Pdo index=\"0x1A04\" is_tx=\"true\">"
        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"pos_following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        "			</Pdo>"
        "			<Pdo index=\"0x1A11\" is_tx=\"true\">"
        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
        "			</Pdo>"
        "			<Pdo index=\"0x1A1C\" is_tx=\"true\">"
        "				<PdoEntry name=\"digital_input\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
        "			</Pdo>"
        "			<Pdo index=\"0x1A1D\" is_tx=\"true\">"
        "				<PdoEntry name=\"analog_input_1\" index=\"0x2205\" subindex=\"0x01\" size=\"16\"/>"
        "			</Pdo>"
        "			<Pdo index=\"0x1A1E\" is_tx=\"true\">"
        "				<PdoEntry name=\"aux_pos_actual_value\" index=\"0x20A0\" subindex=\"0x00\" size=\"32\"/>"
        "			</Pdo>"
        "		</SyncManager>"
        "	</SyncManagerPoolObject>"
        "</EthercatMotion>";

    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_roll);

//    std::string xml_str_link_1 =
//        "<EthercatMotion phy_id=\"1\" product_code=\"0x00030924\""
//        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
//        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
//        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
//        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
//        "	<SyncManagerPoolObject>"
//        "		<SyncManager is_tx=\"false\"/>"
//        "		<SyncManager is_tx=\"true\"/>"
//        "		<SyncManager is_tx=\"false\">"
//        "           <Pdo index=\"0x1605\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"max_tor\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
//        "           </Pdo>"
//        "           <Pdo index=\"0x161D\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x60FE\" subindex=\"0x01\" size=\"32\"/>"
//        "           </Pdo>"
//        "		</SyncManager>"
//        "		<SyncManager is_tx=\"true\">"
//        "			<Pdo index=\"0x1A04\" is_tx=\"true\">"
//        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"pos_following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A11\" is_tx=\"true\">"
//        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1C\" is_tx=\"true\">"
//        "				<PdoEntry name=\"digital_input\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1D\" is_tx=\"true\">"
//        "				<PdoEntry name=\"analog_input_1\" index=\"0x2205\" subindex=\"0x01\" size=\"16\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1E\" is_tx=\"true\">"
//        "				<PdoEntry name=\"aux_pos_actual_value\" index=\"0x20A0\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "		</SyncManager>"
//        "	</SyncManagerPoolObject>"
//        "</EthercatMotion>";

//    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_1);

//    std::string xml_str_link_2 =
//        "<EthercatMotion phy_id=\"2\" product_code=\"0x00030924\""
//        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
//        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
//        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
//        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
//        "	<SyncManagerPoolObject>"
//        "		<SyncManager is_tx=\"false\"/>"
//        "		<SyncManager is_tx=\"true\"/>"
//        "		<SyncManager is_tx=\"false\">"
//        "           <Pdo index=\"0x1605\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"max_tor\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
//        "           </Pdo>"
//        "           <Pdo index=\"0x161D\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x60FE\" subindex=\"0x01\" size=\"32\"/>"
//        "           </Pdo>"
//        "		</SyncManager>"
//        "		<SyncManager is_tx=\"true\">"
//        "			<Pdo index=\"0x1A04\" is_tx=\"true\">"
//        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"pos_following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A11\" is_tx=\"true\">"
//        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1C\" is_tx=\"true\">"
//        "				<PdoEntry name=\"digital_input\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1D\" is_tx=\"true\">"
//        "				<PdoEntry name=\"analog_input_1\" index=\"0x2205\" subindex=\"0x01\" size=\"16\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1E\" is_tx=\"true\">"
//        "				<PdoEntry name=\"aux_pos_actual_value\" index=\"0x20A0\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "		</SyncManager>"
//        "	</SyncManagerPoolObject>"
//        "</EthercatMotion>";

//    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_2);

//    std::string xml_str_link_3 =
//        "<EthercatMotion phy_id=\"3\" product_code=\"0x00030924\""
//        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
//        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
//        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
//        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
//        "	<SyncManagerPoolObject>"
//        "		<SyncManager is_tx=\"false\"/>"
//        "		<SyncManager is_tx=\"true\"/>"
//        "		<SyncManager is_tx=\"false\">"
//        "           <Pdo index=\"0x1605\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//        "               <PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"max_tor\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
//        "               <PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
//        "           </Pdo>"
//        "           <Pdo index=\"0x161D\" is_tx=\"false\">"
//        "               <PdoEntry name=\"target_pos\" index=\"0x60FE\" subindex=\"0x01\" size=\"32\"/>"
//        "           </Pdo>"
//        "		</SyncManager>"
//        "		<SyncManager is_tx=\"true\">"
//        "			<Pdo index=\"0x1A04\" is_tx=\"true\">"
//        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"pos_following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
//        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A11\" is_tx=\"true\">"
//        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1C\" is_tx=\"true\">"
//        "				<PdoEntry name=\"digital_input\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1D\" is_tx=\"true\">"
//        "				<PdoEntry name=\"analog_input_1\" index=\"0x2205\" subindex=\"0x01\" size=\"16\"/>"
//        "			</Pdo>"
//        "			<Pdo index=\"0x1A1E\" is_tx=\"true\">"
//        "				<PdoEntry name=\"aux_pos_actual_value\" index=\"0x20A0\" subindex=\"0x00\" size=\"32\"/>"
//        "			</Pdo>"
//        "		</SyncManager>"
//        "	</SyncManagerPoolObject>"
//        "</EthercatMotion>";

//    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_3);

    return controller;
}

typedef struct _BOX
{
    int    flag;
    char   szMsg[TEST_SIZE];
    int    shm_id;
    void*  shm;
    double num_1;
    double num_2;
}Box;

auto SetVel::prepairNrt(PlanTarget &target)->void
{
    Box box;
    box.shm_id = shmget(13, 2048, IPC_CREAT | 0666);
    box.shm = shmat(box.shm_id, NULL, 0);
    Box *pBox = (Box*)(box.shm);
    target.param = pBox;

    std::vector<std::pair<std::string, std::any>> ret_value;
    target.ret = ret_value;
}

auto SetVel::executeRT(PlanTarget &target)->int
{
    int time = 10000000;
    static double joint_1_old;

    auto &param = std::any_cast<Box*&>(target.param);

    if(param->flag == 1)
    {
        param->flag = 0;
        char* p = param->szMsg;
        sscanf(p,"%lf %lf",&(param->num_1), &(param->num_2));

        if(target.count == 1)
        {
          joint_1_old = param->num_2;
        }
        double joint_1_error = std::abs(param->num_2 - joint_1_old);
        if(joint_1_error > 0.01){
          int number = joint_1_error/0.01;
          for(int i =1; i< number; i++){
            double joint_1_new = joint_1_old + i * 0.01;
            joint_1.push(joint_1_new);
          }
          joint_1.push(param->num_2);
          joint_1_old = param->num_2;
        }else{
          joint_1.push(param->num_2);
          joint_1_old = param->num_2;
        }
    }

    return time - target.count;
}
auto SetVel::collectNrt(PlanTarget &target)->void {}
SetVel::SetVel(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"SetVel\">"
        "</Command>");
}

auto Motion::prepairNrt(PlanTarget &target)->void
{
    for(auto &option:target.mot_options)option|=
            aris::plan::Plan::USE_TARGET_VEL
            |aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS
            |aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR
            |aris::plan::Plan::NOT_CHECK_VEL_MAX
            |aris::plan::Plan::NOT_CHECK_VEL_MIN
            |aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER
            |aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS
            |aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR
            |aris::plan::Plan::NOT_CHECK_POS_MAX
            |aris::plan::Plan::NOT_CHECK_POS_MIN;

    std::vector<std::pair<std::string, std::any>> ret_value;
    target.ret = ret_value;
}

auto Motion::executeRT(PlanTarget &target)->int
{
  static double joint_1_old;
  int time = 10000000;
  if(!joint_1.empty()){
    double joint_1_target = joint_1.front();
    if(target.count == 1)
    {
      joint_1_old = joint_1_target;
    }
    double error = std::abs(joint_1_target - joint_1_old);
    joint_1_old = joint_1_target;
    printf("error = %lf\n", error);
    joint_1.pop();
  }
  return time - target.count;
}

auto Motion::collectNrt(PlanTarget &target)->void {}
Motion::Motion(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"Motion\">"
        "</Command>");
}

//  planPool //
auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Recover>();
    plan_root->planPool().add<aris::plan::Reset>();
    plan_root->planPool().add<SetVel>();
//    plan_root->planPool().add<Motion>();
    return plan_root;
}

int main(int argc, char *argv[])
{
    //creat Ethercat object
    aris::control::EthercatMaster mst;
    mst.scan();
    std::cout<<"slave num:"<<mst.slavePool().size()<<std::endl;

    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());
    std::cout<<"start controller server"<<std::endl;
    //thread start
    cs.start();
    int16_t torque = 1000;
    auto &cm_1 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(0));
//    auto &cm_2 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(1));
//    auto &cm_3 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(2));
//    auto &cm_4 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(3));
    cm_1.writePdo(0x6072, 0x00, &torque, 16);
//    cm_2.writePdo(0x6072, 0x00, &torque, 16);
//    cm_3.writePdo(0x6072, 0x00, &torque, 16);
//    cm_4.writePdo(0x6072, 0x00, &torque, 16);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    //8:position mode,9:velocity mode,10:current mode
    cs.executeCmd(aris::core::Msg("ds"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("md --mode=8"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("en"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("SetVel"));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    cs.executeCmd(aris::core::Msg("Motion"));
    //Receive Command//
    cout<< "ok" <<std::endl;
    cs.runCmdLine();
}













































