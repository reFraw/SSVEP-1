#include <ros/ros.h>
#include <robotics_function/main_robotics_function.h>

#include <ssvep1/sendInitState.h>

#include <thesis_msgs/FingerJoints.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <kinova_msgs/JointVelocity.h>

#include <kinova_driver/kinova_fingers_action.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Robot
{
    private:

        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<kinova_msgs::SetFingersPositionAction> fingerServer_;

        ros::Publisher jointStatePublisher_;
        ros::Publisher kinovaPublisher_;
        ros::Subscriber jointVelocitiesSubscriber_;

        ros::Rate ROBOT_CLOCK{100};

        bool INIT_STATE_RECEIVED_ = false;

        double SAMPLING_TIME_ = 0.01;

        sensor_msgs::JointState JOINT_MSG_;
        sensor_msgs::JointState KINOVA_MSG_;
        kinova_msgs::SetFingersPositionResult FINGER_RES_;

        std::vector<double> JOINT_STATE_ = {2.912, 0.427, 0.665, 1.262, -0.323, 0.943, 5.240};
        std::vector<double> FINGER_STATE_ = {0, 0, 0, 0, 0, 0};
        std::vector<std::string>  JOINT_NAMES_ = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7",
        };
        std::vector<std::string> FINGER_NAMES_ ={
            "j2s7s300_joint_finger_1",
            "j2s7s300_joint_finger_2",
            "j2s7s300_joint_finger_3",
            "j2s7s300_joint_finger_tip_1",
            "j2s7s300_joint_finger_tip_2",
            "j2s7s300_joint_finger_tip_3"
        };

        std::vector<double> integrate(std::vector<double> velocities)
        {
            int nJoints = velocities.size();
            std::vector<double> newConfiguration(nJoints);

            for(int i=0; i<nJoints; i++)
            {
                newConfiguration[i] = JOINT_STATE_[i] + SAMPLING_TIME_*velocities[i];
            }

            return newConfiguration;
        };

        void velocitiesCallback(const kinova_msgs::JointVelocityConstPtr& velocitiesMsg)
        {
            std::vector<double> velocities = {
                velocitiesMsg->joint1,
                velocitiesMsg->joint2,
                velocitiesMsg->joint3,
                velocitiesMsg->joint4,
                velocitiesMsg->joint5,
                velocitiesMsg->joint6,
                velocitiesMsg->joint7
            };

            velocities = convertToRad(velocities);
            velocities = convertToDHVelocities(velocities);

            JOINT_STATE_ = integrate(velocities);
        };

        void executeFingerCallback(const kinova_msgs::SetFingersPositionGoalConstPtr& goal)
        {
            FINGER_RES_.fingers.finger1 = goal->fingers.finger1;
            FINGER_RES_.fingers.finger2 = goal->fingers.finger2;
            FINGER_RES_.fingers.finger3 = goal->fingers.finger3;

            fingerServer_.setSucceeded(FINGER_RES_);
        }


    public:

        Robot() : 
            fingerServer_(nh_, "finger", boost::bind(&Robot::executeFingerCallback, this, _1), false)
        {
            ros::param::get("/SAMPLING_TIME", SAMPLING_TIME_);

            kinovaPublisher_ = nh_.advertise<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1);
            jointVelocitiesSubscriber_ = nh_.subscribe("/j2s7s300_driver/in/joint_velocity", 1, &Robot::velocitiesCallback, this);

            fingerServer_.start();

            KINOVA_MSG_.name = JOINT_NAMES_;

            while(ros::ok())
            {
                
                std::vector<double> kinovajoints = convertToJaco2(JOINT_STATE_);
                KINOVA_MSG_.position = kinovajoints;

                KINOVA_MSG_.header.stamp = ros::Time::now();

                kinovaPublisher_.publish(KINOVA_MSG_);

                ros::spinOnce();
                ROBOT_CLOCK.sleep();
            }
        };

        ~Robot(){};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco2");

    Robot Robot;

    return 0;
}