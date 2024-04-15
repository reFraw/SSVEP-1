#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "ssvep1/sendInitState.h"

class InitStateSender {

    private:
        ros::NodeHandle nh_;
        ros::ServiceServer initServer_;

        double JOINT_1_;
        double JOINT_2_;
        double JOINT_3_;
        double JOINT_4_;
        double JOINT_5_;
        double JOINT_6_;
        double JOINT_7_;

        bool sendInitJointState(ssvep1::sendInitState::Request& req, ssvep1::sendInitState::Response& res) {

            std::vector<double> initJointState = {
                JOINT_1_,
                JOINT_2_,
                JOINT_3_,
                JOINT_4_,
                JOINT_5_,
                JOINT_6_,
                JOINT_7_
            };

            res.initJointState = initJointState;
            ROS_INFO("Initial joint state sended");
            return true;

        }

    public:

        InitStateSender() {

            if (nh_.hasParam("/JOINT_1")) {
                ros::param::get("/JOINT_1", JOINT_1_);
            }
            else {
                JOINT_1_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_2")) {
                ros::param::get("/JOINT_2", JOINT_2_);
            }
            else {
                JOINT_2_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_3")) {
                ros::param::get("/JOINT_3", JOINT_3_);
            }
            else {
                JOINT_3_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_4")) {
                ros::param::get("/JOINT_4", JOINT_4_);
            }
            else {
                JOINT_4_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_5")) {
                ros::param::get("/JOINT_5", JOINT_5_);
            }
            else {
                JOINT_5_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_6")) {
                ros::param::get("/JOINT_6", JOINT_6_);
            }
            else {
                JOINT_6_ = 0.0;
            }

            if (nh_.hasParam("/JOINT_7")) {
                ros::param::get("/JOINT_7", JOINT_7_);
            }
            else {
                JOINT_7_ = 0.0;
            }
            

            initServer_ = nh_.advertiseService("/initial_joint_state", &InitStateSender::sendInitJointState, this);
            ROS_INFO("Ready to send initial joint state");
            ros::spin();

        }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "init_state_server");

    InitStateSender InitStateSender;

    return 0;

} 