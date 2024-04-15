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

class Robot {

    private:
        ros::NodeHandle nh_;

        ros::Publisher jointStatePub_;
        ros::Publisher kinovaPublisher_;
        ros::Subscriber jointVelocitySub_;

        ros::ServiceClient initStateClient_;

        bool INIT_STATE_RECEIVED = false;

        double SAMPLING_TIME_ = 0.01;

        std::vector<double> JOINT_STATE_{7};

        std::vector<std::string>  JOINT_NAMES = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7",
        };

        void jointVelocityCallback(const sensor_msgs::JointStateConstPtr& velocityMsg) {

            std::vector<double> dq = velocityMsg->velocity;
            std::vector<double> newQ = integrateVelocities(dq, JOINT_STATE_);

            JOINT_STATE_ = newQ;

            sensor_msgs::JointState jointMsg;
            sensor_msgs::JointState kinovaMsg;

            jointMsg.name = JOINT_NAMES;
            kinovaMsg.name = JOINT_NAMES;

            jointMsg.position = JOINT_STATE_;
            kinovaMsg.position = convertToJaco2(JOINT_STATE_);

            jointMsg.header.stamp = ros::Time::now();
            kinovaMsg.header.stamp = ros::Time::now();

            jointStatePub_.publish(jointMsg);
            kinovaPublisher_.publish(kinovaMsg);

        }

        std::vector<double> integrateVelocities(std::vector<double> jointVelocities, std::vector<double> jointState) {

            int nJoints = jointVelocities.size();

            std::vector<double> newQ(nJoints);
            Eigen::VectorXd q(nJoints);
            Eigen::VectorXd dq(nJoints);

            for(int i = 0; i < nJoints; i++) {
                q[i] = jointState[i];
                dq[i] = jointVelocities[i];
            }

            Eigen::VectorXd qNext = q + SAMPLING_TIME_*dq;

            for(int i = 0; i < nJoints; i++) {
                newQ[i] = qNext[i];
            }

            return newQ;

        }


    public:

        Robot() {

            ros::param::get("/SAMPLING_TIME", SAMPLING_TIME_);

            initStateClient_ = nh_.serviceClient<ssvep1::sendInitState>("/initial_joint_state");
            ssvep1::sendInitState initSrv;

            while(!INIT_STATE_RECEIVED) {

                if(initStateClient_.call(initSrv)) {
                    JOINT_STATE_ = initSrv.response.initJointState;
                    ROS_INFO("Initial joint state received");
                    INIT_STATE_RECEIVED = true;
                }
                else{
                    ROS_WARN("Waiting for initial state");
                }

                ros::Rate{100}.sleep();

            }

            jointStatePub_ = nh_.advertise<sensor_msgs::JointState>("/joint_state", 1);
            kinovaPublisher_ = nh_.advertise<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1);
            jointVelocitySub_ = nh_.subscribe("/joint_velocities", 1, &Robot::jointVelocityCallback, this);

            ros::spin();
        }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "robot");

    Robot Robot;

    return 0;

}