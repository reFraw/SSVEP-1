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

class Robot
{
    private:

        ros::NodeHandle nh_;

        ros::ServiceClient initStateClient_;

        ros::Publisher jointStatePublisher_;
        ros::Publisher kinovaPublisher_;
        ros::Subscriber jointVelocitiesSubscriber_;

        ros::Rate ROBOT_CLOCK{100};

        bool INIT_STATE_RECEIVED_ = false;

        double SAMPLING_TIME_ = 0.01;

        sensor_msgs::JointState JOINT_MSG_;
        sensor_msgs::JointState KINOVA_MSG_;

        std::vector<double> JOINT_STATE_{7};
        std::vector<std::string>  JOINT_NAMES_ = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7",
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

        void velocitiesCallback(const sensor_msgs::JointStateConstPtr& velocitiesMsg)
        {
            std::vector<double> velocities = velocitiesMsg->velocity;
            JOINT_STATE_ = integrate(velocities);
        };

    public:

        Robot()
        {
            ros::param::get("/SAMPLING_TIME", SAMPLING_TIME_);

            initStateClient_ = nh_.serviceClient<ssvep1::sendInitState>("/initial_joint_state");
            ssvep1::sendInitState initStateSrv;

            while(!INIT_STATE_RECEIVED_)
            {
                if(initStateClient_.call(initStateSrv))
                {
                    JOINT_STATE_ = initStateSrv.response.initJointState;
                    ROS_INFO("Initial joint configuration received.");
                    INIT_STATE_RECEIVED_ = true;
                }
                else{
                    ROS_WARN("Waiting to receive the initial joint configuration.");
                }

                ROBOT_CLOCK.sleep();
            }

            initStateClient_.shutdown();

            jointStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_state", 1);
            kinovaPublisher_ = nh_.advertise<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1);
            jointVelocitiesSubscriber_ = nh_.subscribe("/joint_velocities", 1, &Robot::velocitiesCallback, this);

            JOINT_MSG_.name = JOINT_NAMES_;
            KINOVA_MSG_.name = JOINT_NAMES_;

            while(ros::ok())
            {
                JOINT_MSG_.position = JOINT_STATE_;
                KINOVA_MSG_.position = convertToJaco2(JOINT_STATE_);

                JOINT_MSG_.header.stamp = ros::Time::now();
                KINOVA_MSG_.header.stamp = ros::Time::now();

                jointStatePublisher_.publish(JOINT_MSG_);
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