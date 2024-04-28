#include "ros/ros.h"

#include "robotics_function/main_robotics_function.h"

#include "thesis_msgs/DesiredWaypoint.h"
#include "thesis_msgs/Error.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

#include "ssvep1/sendInitState.h"
#include "ssvep1/sendInitPose.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

class Controller
{
    private:

        ros::NodeHandle nh_;

        ros::ServiceClient initJointStateClient_;
        ros::ServiceServer initPoseServer_;

        ros::Publisher jointVelocitiesPublisher_;
        ros::Publisher errorPublisher_;
        ros::Publisher currentPosePublisher_; 
        ros::Subscriber waypointSubscriber_;
        ros::Subscriber jointStateSubscriber_;

        ros::Rate CONTROLLER_CLOCK_{100};

        std::vector<double> JOINT_STATE_{7};
        std::vector<std::string> JOINT_NAMES_ = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7"
        };

        double GAIN_ = 5;

        bool INIT_STATE_RECEIVED_ = false;
        bool POSE_SENDED_ = false;

        bool ENABLE_MANIPULABILITY_ = false;
        double K0_;
        double DERIVATIVE_STEP_;

        Eigen::Vector3d DESIRED_POSITION_;
        Eigen::Quaterniond DESIRED_ORIENTATION_;
        Eigen::Vector3d DESIRED_LINEAR_VELOCITY_ = {0.00, 0.00, 0.00};
        Eigen::Vector3d DESIRED_ANGULAR_VELOCITY_ = {0.00, 0.00, 0.00};

        geometry_msgs::Pose CURRENT_POSE_MSG_;
        thesis_msgs::Error ERROR_MSG_;
        sensor_msgs::JointState JOINT_VELOCITIES_MSG_;

        void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointMsg)
        {
            JOINT_STATE_ = jointMsg->position;

            Eigen::Matrix4d T = directKinematics(JOINT_STATE_, JOINT_STATE_.size());
            Eigen::Quaterniond Q(T.block<3,3>(0,0));

            CURRENT_POSE_MSG_.position.x = T(0,3);
            CURRENT_POSE_MSG_.position.y = T(1,3);
            CURRENT_POSE_MSG_.position.z = T(2,3);
            CURRENT_POSE_MSG_.orientation.w = Q.w();
            CURRENT_POSE_MSG_.orientation.x = Q.x();
            CURRENT_POSE_MSG_.orientation.y = Q.y();
            CURRENT_POSE_MSG_.orientation.z = Q.z();

            currentPosePublisher_.publish(CURRENT_POSE_MSG_);
        };

        void waypointCallback(const thesis_msgs::DesiredWaypointConstPtr& waypointMsg)
        {
            DESIRED_POSITION_ = Eigen::Vector3d(
                waypointMsg->position.x,
                waypointMsg->position.y,
                waypointMsg->position.z
            );

            DESIRED_ORIENTATION_ = Eigen::Quaterniond(
                waypointMsg->orientation.w,
                waypointMsg->orientation.x,
                waypointMsg->orientation.y,
                waypointMsg->orientation.z
            );

            DESIRED_LINEAR_VELOCITY_ = Eigen::Vector3d(
                waypointMsg->linearVelocity.x,
                waypointMsg->linearVelocity.y,
                waypointMsg->linearVelocity.z
            );

            DESIRED_ANGULAR_VELOCITY_ = Eigen::Vector3d(
                waypointMsg->angularVelocity.x,
                waypointMsg->angularVelocity.y,
                waypointMsg->angularVelocity.z
            );
        };

        bool sendInitPose(ssvep1::sendInitPose::Request& req, ssvep1::sendInitPose::Response& res)
        {
            Eigen::Matrix4d T = directKinematics(JOINT_STATE_,JOINT_STATE_.size());
            Eigen::Quaterniond orientationQuat(T.block<3,3>(0,0));

            std::vector<double> position ={
                T(0,3),
                T(1,3),
                T(2,3)
            };
            std::vector<double> orientation = {
                orientationQuat.w(),
                orientationQuat.x(),
                orientationQuat.y(),
                orientationQuat.z()
            };

            res.position = position;
            res.orientation = orientation;

            ROS_INFO("Initial manipulator pose sent.");
            POSE_SENDED_ = true;

            return true;
        };

    public:

        Controller()
        {
            ros::param::get("/GAIN", GAIN_);
            ros::param::get("/ENABLE_MANIPULABILITY", ENABLE_MANIPULABILITY_);
            ros::param::get("/NULL_GAIN", K0_);
            ros::param::get("/DERIVATIVE_STEP", DERIVATIVE_STEP_);

            JOINT_VELOCITIES_MSG_.name = JOINT_NAMES_;

            initJointStateClient_ = nh_.serviceClient<ssvep1::sendInitState>("/initial_joint_state");
            ssvep1::sendInitState initStateSrv;

            while(!INIT_STATE_RECEIVED_)
            {
                if(initJointStateClient_.call(initStateSrv))
                {
                    JOINT_STATE_ = initStateSrv.response.initJointState;

                    Eigen::Matrix4d T = directKinematics(JOINT_STATE_, JOINT_STATE_.size());

                    DESIRED_POSITION_ = {T(0,3), T(1,3), T(2,3)};
                    DESIRED_ORIENTATION_ = Eigen::Quaterniond(T.block<3,3>(0,0));

                    INIT_STATE_RECEIVED_ = true;
                    ROS_INFO("Initial joint configuration received.");
                }
                else
                {
                    ROS_WARN("Waiting to receive the initial joint configuration.");
                }

                CONTROLLER_CLOCK_.sleep();
            }

            initPoseServer_ = nh_.advertiseService("/initial_pose", &Controller::sendInitPose, this);

            jointVelocitiesPublisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_velocities", 1);
            errorPublisher_ = nh_.advertise<thesis_msgs::Error>("/errors", 1);
            currentPosePublisher_ = nh_.advertise<geometry_msgs::Pose>("/current_pose", 1);
            jointStateSubscriber_ = nh_.subscribe("/joint_state", 1, &Controller::jointStateCallback, this);
            waypointSubscriber_ = nh_.subscribe("/desired_waypoint", 1, &Controller::waypointCallback, this);

            while(ros::ok())
            {   
                std::vector<std::vector<double>> results(2);

                if(ENABLE_MANIPULABILITY_)
                {
                    Eigen::VectorXd nullVelocity = computeNullVelocity(JOINT_STATE_, DERIVATIVE_STEP_, K0_);
                    results = applyIK2withNull(DESIRED_POSITION_, DESIRED_ORIENTATION_, DESIRED_LINEAR_VELOCITY_, DESIRED_ANGULAR_VELOCITY_, JOINT_STATE_, GAIN_, nullVelocity);
                }
                else
                {
                    results = applyIK2(DESIRED_POSITION_, DESIRED_ORIENTATION_, DESIRED_LINEAR_VELOCITY_, DESIRED_ANGULAR_VELOCITY_, JOINT_STATE_, GAIN_);
                }

                std::vector<double> velocities = results[0];
                std::vector<double> errors = results[1];

                ERROR_MSG_.positionErrorX = errors[0];
                ERROR_MSG_.positionErrorY = errors[1];
                ERROR_MSG_.positionErrorZ = errors[2];
                ERROR_MSG_.orientationErrorX = errors[3];
                ERROR_MSG_.orientationErrorY = errors[4];
                ERROR_MSG_.orientationErrorZ = errors[5];

                JOINT_VELOCITIES_MSG_.header.stamp = ros::Time::now();
                JOINT_VELOCITIES_MSG_.velocity = velocities;

                jointVelocitiesPublisher_.publish(JOINT_VELOCITIES_MSG_);
                errorPublisher_.publish(ERROR_MSG_);

                ros::spinOnce();
                CONTROLLER_CLOCK_.sleep();
            }

        };

        ~Controller(){};
    
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "controller");
    
    Controller Controller;

    return 0;

}