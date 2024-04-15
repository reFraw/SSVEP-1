#include <ros/ros.h>
#include <robotics_function/main_robotics_function.h>

#include <ssvep1/sendInitState.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Interface {

    private:
        ros::NodeHandle nh_;

        ros::Publisher instrPub_;
        ros::Subscriber readySub_;

        bool READY_TO_SEND = true;

        void readyCallback(const std_msgs::BoolConstPtr& readyMsg) {
            READY_TO_SEND = readyMsg->data;
        }

        std_msgs::String acquireInstruction() {

            std_msgs::String instrMsg;
            std::string instr;

            std::cout << "[INFO] Insert instruction >> ";
            std::cin >> instr;

            instrMsg.data = instr;

            return instrMsg;
        }

    public:

        Interface() {

            system("clear");

            readySub_ = nh_.subscribe("/ready", 1, &Interface::readyCallback, this);
            instrPub_ = nh_.advertise<std_msgs::String>("/instructions", 1);

            while(ros::ok()) {

                if(READY_TO_SEND) {
                    std_msgs::String msg = acquireInstruction();
                    instrPub_.publish(msg);
                    READY_TO_SEND = false;
                }

                ros::Rate{100}.sleep();
                ros::spinOnce();

            }

        }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "interface");

    Interface Interface;

    return 0;

}