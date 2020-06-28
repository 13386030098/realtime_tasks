#include <iostream>
#include <aris.hpp>
#include "moveEA.h"
#include <Eigen/Eigen>
#include <math.h>
#include <string>
#include <signal.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define TEST_SIZE 2048
using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

#define pi M_PI
typedef void(*signal_handler);

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
    //4096 represent 4096 counts/revolution
    double pos_factor_roll = 4096.0*120/2/PI;
    double pos_factor_link = 4096.0*100/2/PI;


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

    std::string xml_str_link_1 =
        "<EthercatMotion phy_id=\"1\" product_code=\"0x00030924\""
        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
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

    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_1);

    std::string xml_str_link_2 =
        "<EthercatMotion phy_id=\"2\" product_code=\"0x00030924\""
        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
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

    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_2);

    std::string xml_str_link_3 =
        "<EthercatMotion phy_id=\"3\" product_code=\"0x00030924\""
        " vendor_id=\"0x9A\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
        " min_pos=\"-3.14\" max_pos=\"3.14\" max_vel=\"0.5\" min_vel=\"-0.5\""
        " max_acc=\"5\" min_acc=\"-5\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
        " home_pos=\"0\" pos_factor=\""+std::to_string(pos_factor_link)+"\" pos_offset=\"0.0\">"
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

    controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str_link_3);

    return controller;
};

typedef struct _BOX
{
    int  flag;
    char szMsg[TEST_SIZE];
    int shm_id;
    void* shm;
    double position_test;
    double num_1, num_2;
    int i;
}Box;

auto SetVel::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    Box box;
    box.shm_id = shmget(13, 2048, IPC_CREAT | 0666);
    box.shm = shmat(box.shm_id, NULL, 0);
    Box *pBox = (Box*)(box.shm);
    target.param = pBox;

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

auto SetVel::executeRT(PlanTarget &target)->int
{
//    clock_t start, end;
//    start = clock();
    int time = 10000000;
    int time_ = 5000;

    auto &param = std::any_cast<Box*&>(target.param);
    auto&cs = aris::server::ControlServer::instance();
    // 访问主站 //
    auto &controller = target.controller;
    auto &cout = controller->mout();
    auto &lout = controller->lout();
    static double beginpos[1];
    static double target_Pos[4];

    if(target.count == 1)
    {
        beginpos[0] = controller->motionPool()[0].actualPos();
    }
//    cout<< controller->motionPool()[0].actualPos() <<std::endl;
    if(param->flag == 1)
    {
        param->flag = 0;
        char* p = param->szMsg;
        printf("msg from writer is [%s]\n", p);
        sscanf(p,"%lf %lf",&(param->num_1), &(param->num_2));
        printf("num1 = %lf\n",param->num_1);
        printf("num2 = %lf\n",param->num_2);

        controller->motionPool()[0].setTargetPos(param->num_2);
        controller->motionPool()[1].setTargetPos(param->num_1);
        controller->motionPool()[2].setTargetPos(-param->num_1);
        controller->motionPool()[3].setTargetPos(param->num_1);

//        lout <<controller->motionPool()[0].actualPos()<<std::endl;
////        target_Pos[0] = atof(param->szMsg);

//        target_Pos[0] = beginpos[0] + 0.7*(1-std::cos(2 * 3.14*target.count / time_));
////        controller->motionPool()[0].setTargetPos(target_Pos[0]);
//        cout << "target: " << target_Pos[0] <<std::endl;
//        cout << "actual: "<< controller->motionPool()[0].actualPos()<<std::endl;
    }

//    static double target_vel = 0.0;
//    static double target_Pos = 0.0;
//    static double TargetToq = 0.0;

//    target_vel =-0.1;
//    TargetToq = 200;
//    target_Pos = controller->motionPool()[0].actualPos() + 0.01;
//    controller->motionAtAbs(0).setTargetToq(TargetToq);
//    controller->motionAtAbs(0).setTargetVel(target_vel);
//    controller->motionPool()[0].setTargetPos(target_Pos);
//    cout<< controller->motionPool()[0].actualPos() <<std::endl;

//    cout<< controller->motionPool()[0].actualToq() <<std::endl;
//    lout <<controller->motionPool()[0].actualToq() <<std::endl;
//    end = clock();
//    cout<< (double)(end - start)/CLOCKS_PER_SEC<<endl;
    return time - target.count;
}
auto SetVel::collectNrt(PlanTarget &target)->void {}
SetVel::SetVel(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"SetVel\">"
        "	<GroupParam>"
        "		<Param name=\"vel\" default=\"0.1\"/>"
        "	</GroupParam>"
        "</Command>");
}

// 将创建的轨迹添加到轨迹规划池planPool中 //
auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.0}");
    plan_root->planPool().add<SetVel>();
    return plan_root;
}

// 主函数

int main(int argc, char *argv[])
{
    //创建Ethercat主站对象
    aris::control::EthercatMaster mst;
   // 自动扫描，连接从站
    mst.scan();
//    std::cout<<mst.xmlString()<<std::endl;
   // 打印主站扫描的从站个数
//   std::cout<<"slave num:"<<mst.slavePool().size()<<std::endl;
    //8:位置模式,9:速度模式,10:电流模式
    //从站的顺序与实际物理拓扑相同
    //cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体

    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());
    std::cout<<"start controller server"<<std::endl;
    //启动线程
    cs.start();
    int16_t torque = 1000;
    auto &cm_1 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(0));
    auto &cm_2 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(1));
    auto &cm_3 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(2));
    auto &cm_4 = dynamic_cast<aris::control::EthercatMotion &>(cs.controller().motionPool().at(3));
    cm_1.writePdo(0x6072, 0x00, &torque, 16);
    cm_2.writePdo(0x6072, 0x00, &torque, 16);
    cm_3.writePdo(0x6072, 0x00, &torque, 16);
    cm_4.writePdo(0x6072, 0x00, &torque, 16);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    //8：位置模式， 9：速度模式， 10：电流模式
    cs.executeCmd(aris::core::Msg("ds"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("md --mode=8"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("en"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cs.executeCmd(aris::core::Msg("SetVel"));

    //Receive Command//
    cout<< "ok" <<std::endl;
    cs.runCmdLine();
}













































