#include "ros/ros.h"
#include <robotics_function/main_robotics_function.h>

#include <actionlib/client/simple_action_client.h>

#include "ssvep1/GrabAction.h"
#include "ssvep1/MoveAction.h"
#include "ssvep1/ReleaseAction.h"
#include "ssvep1/GrabAndReleaseAction.h"

class Interface
{
    private:
        ros::NodeHandle nh_;

        ros::Rate INTERFACE_CLOCK_{100};

        actionlib::SimpleActionClient<ssvep1::GrabAction> grabClient_;
        actionlib::SimpleActionClient<ssvep1::MoveAction> moveClient_;
        actionlib::SimpleActionClient<ssvep1::ReleaseAction> releaseClient_;
        actionlib::SimpleActionClient<ssvep1::GrabAndReleaseAction> grabAndReleaseClient_;

        ssvep1::MoveGoal MOVE_GOAL_;
        ssvep1::GrabGoal GRAB_GOAL_;
        ssvep1::ReleaseGoal RELEASE_GOAL_;
        ssvep1::GrabAndReleaseGoal GRAB_AND_REL_GOAL_;

        std::string INSTRUCTION_;
        std::string INSTR_TYPE_;
        std::vector<std::string> INSTR_COMPS_;

        std::vector<std::string> splitString(std::string input)
        {
            std::vector<std::string> result;
            std::istringstream iss(input);
            std::string segment;

            while (std::getline(iss, segment, '|'))
            {
                std::istringstream segmentStream(segment);
                std::string element;

                while (std::getline(segmentStream, element, ','))
                {
                    result.push_back(element);
                }
            }

            return result;
        };

        std::string acquireInstruction()
        {
            std::string instruction;

            std::cout << "[READY] Insert instruction >> ";
            std::cin >> instruction;

            return instruction;
        };
        

    public:

        Interface() :
            moveClient_("Move", true),
            grabClient_("Grab", true),
            releaseClient_("Release", true),
            grabAndReleaseClient_("GrabRel", true)
        {
            system("clear");

            while(ros::ok())
            {
                INSTRUCTION_ = acquireInstruction();
                INSTR_COMPS_ = splitString(INSTRUCTION_);
                INSTR_TYPE_ = INSTR_COMPS_[0];

                if(INSTR_TYPE_ == "MOVE")
                {
                    MOVE_GOAL_.displacement = {
                        std::stod(INSTR_COMPS_[1]),
                        std::stod(INSTR_COMPS_[2]),
                        std::stod(INSTR_COMPS_[3])
                    };
                    MOVE_GOAL_.rpy = {
                        std::stod(INSTR_COMPS_[4]),
                        std::stod(INSTR_COMPS_[5]),
                        std::stod(INSTR_COMPS_[6])
                    };
                    MOVE_GOAL_.time = std::stod(INSTR_COMPS_[7]);
                    MOVE_GOAL_.velocity = std::stod(INSTR_COMPS_[8]);
                    ROS_INFO("Sendind goal.");
                    moveClient_.sendGoal(MOVE_GOAL_);
                    ROS_INFO("Waiting for results.");
                    moveClient_.waitForResult();
                    actionlib::SimpleClientGoalState state = moveClient_.getState();
                    ROS_INFO("Action finished: %s", state.toString().c_str());
                }
                else if(INSTR_TYPE_ == "GRAB")
                {
                    GRAB_GOAL_.object_position = {
                        std::stod(INSTR_COMPS_[1]),
                        std::stod(INSTR_COMPS_[2]),
                        std::stod(INSTR_COMPS_[3])
                    };
                    ROS_INFO("Sendind goal.");
                    grabClient_.sendGoal(GRAB_GOAL_);
                    ROS_INFO("Waiting for results.");
                    grabClient_.waitForResult();
                    actionlib::SimpleClientGoalState state = grabClient_.getState();
                    ROS_INFO("Action finished: %s", state.toString().c_str());
                }
                else if(INSTR_TYPE_ == "REL")
                {
                    RELEASE_GOAL_.release_position = {
                        std::stod(INSTR_COMPS_[1]),
                        std::stod(INSTR_COMPS_[2]),
                        std::stod(INSTR_COMPS_[3])
                    };
                    ROS_INFO("Sendind goal.");
                    releaseClient_.sendGoal(RELEASE_GOAL_);
                    ROS_INFO("Waiting for results.");
                    releaseClient_.waitForResult();
                    actionlib::SimpleClientGoalState state = releaseClient_.getState();
                    ROS_INFO("Action finished: %s", state.toString().c_str());
                }
                else if(INSTR_TYPE_ == "GRABREL")
                {
                    GRAB_AND_REL_GOAL_.object_position = {
                        std::stod(INSTR_COMPS_[1]),
                        std::stod(INSTR_COMPS_[2]),
                        std::stod(INSTR_COMPS_[3])
                    };

                    GRAB_AND_REL_GOAL_.release_position = {
                        std::stod(INSTR_COMPS_[4]),
                        std::stod(INSTR_COMPS_[5]),
                        std::stod(INSTR_COMPS_[6])
                    };
                    ROS_INFO("Sendind goal.");
                    grabAndReleaseClient_.sendGoal(GRAB_AND_REL_GOAL_);
                    ROS_INFO("Waiting for results.");
                    grabAndReleaseClient_.waitForResult();
                    actionlib::SimpleClientGoalState state = grabAndReleaseClient_.getState();
                    ROS_INFO("Action finished: %s", state.toString().c_str());
                }
                else
                {
                    ROS_WARN("Unknown instruction. Retry.");
                }

                ros::spinOnce();
                INTERFACE_CLOCK_.sleep();
            }
        };

        ~Interface(){};

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface");

    Interface Interface;

    return 0;
}