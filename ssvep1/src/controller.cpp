#include "ros/ros.h"

#include "robotics_function/main_robotics_function.h"

#include "thesis_msgs/DesiredWaypoint.h"
#include "thesis_msgs/Error.h"
#include "thesis_msgs/FingerJoints.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

#include "ssvep1/sendInitState.h"
#include "ssvep1/sendInitPose.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "kinova_msgs/JointVelocity.h"

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
        std::vector<double> FINGER_VEL_{6};
        std::vector<std::string> JOINT_NAMES_ = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7"
        };

        double GAIN_ = 2;

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
        kinova_msgs::JointVelocity JOINT_VELOCITIES_MSG_;

        void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointMsg)
        {

            std::vector<double> joint = {
                jointMsg->position[0],
                jointMsg->position[1],
                jointMsg->position[2],
                jointMsg->position[3],
                jointMsg->position[4],
                jointMsg->position[5],
                jointMsg->position[6]
            };

            JOINT_STATE_ = convertToDH(joint);

            Eigen::Matrix4d T = directKinematics(JOINT_STATE_, JOINT_STATE_.size());
            Eigen::Quaterniond Q(T.block<3,3>(0,0));

            CURRENT_POSE_MSG_.position.x = T(0,3);
            CURRENT_POSE_MSG_.position.y = T(1,3);
            CURRENT_POSE_MSG_.position.z = T(2,3);
            CURRENT_POSE_MSG_.orientation.w = Q.w();
            CURRENT_POSE_MSG_.orientation.x = Q.x();
            CURRENT_POSE_MSG_.orientation.y = Q.y();
            CURRENT_POSE_MSG_.orientation.z = Q.z();

            if (!INIT_STATE_RECEIVED_)
            {
                DESIRED_POSITION_ = {T(0,3), T(1,3), T(2,3)};
                DESIRED_ORIENTATION_ = Q;
                INIT_STATE_RECEIVED_ = true;
            }

            currentPosePublisher_.publish(CURRENT_POSE_MSG_);
        };

        void fingerCallback(const thesis_msgs::FingerJointsConstPtr& fingerMsg)
        {
            FINGER_VEL_ = fingerMsg->values;
        }

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

            jointStateSubscriber_ = nh_.subscribe("/j2s7s300_driver/out/joint_state", 1, &Controller::jointStateCallback, this);

            jointVelocitiesPublisher_ = nh_.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity", 1);
            errorPublisher_ = nh_.advertise<thesis_msgs::Error>("/errors", 1);
            currentPosePublisher_ = nh_.advertise<geometry_msgs::Pose>("/current_pose", 1);
            
            waypointSubscriber_ = nh_.subscribe("/desired_waypoint", 1, &Controller::waypointCallback, this);

            while(!INIT_STATE_RECEIVED_)
            {
                ROS_WARN("Waiting for initial state");
                ros::spinOnce();
                CONTROLLER_CLOCK_.sleep();
            }

            initPoseServer_ = nh_.advertiseService("/initial_pose", &Controller::sendInitPose, this);

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

                std::vector<double> velocities = convertToJaco2Velocities(results[0]);
                velocities = convertToDeg(velocities);

                std::vector<double> errors = results[1];

                Eigen::Vector3d posErr = {errors[0], errors[1], errors[2]};
                Eigen::Vector3d oriErr = {errors[3], errors[4], errors[5]};

                // if (posErr.norm() >= 0.2 || oriErr.norm() >= 0.2)
                // {
                //     JOINT_VELOCITIES_MSG_.joint1 = 0;
                //     JOINT_VELOCITIES_MSG_.joint2 = 0;
                //     JOINT_VELOCITIES_MSG_.joint3 = 0;
                //     JOINT_VELOCITIES_MSG_.joint4 = 0;
                //     JOINT_VELOCITIES_MSG_.joint5 = 0;
                //     JOINT_VELOCITIES_MSG_.joint6 = 0;
                //     JOINT_VELOCITIES_MSG_.joint7 = 0;

                //     jointVelocitiesPublisher_.publish(JOINT_VELOCITIES_MSG_);
                //     ros::shutdown();
                // }

                ERROR_MSG_.positionErrorX = errors[0];
                ERROR_MSG_.positionErrorY = errors[1];
                ERROR_MSG_.positionErrorZ = errors[2];
                ERROR_MSG_.orientationErrorX = errors[3];
                ERROR_MSG_.orientationErrorY = errors[4];
                ERROR_MSG_.orientationErrorZ = errors[5];

                JOINT_VELOCITIES_MSG_.joint1 = velocities[0];
                JOINT_VELOCITIES_MSG_.joint2 = velocities[1];
                JOINT_VELOCITIES_MSG_.joint3 = velocities[2];
                JOINT_VELOCITIES_MSG_.joint4 = velocities[3];
                JOINT_VELOCITIES_MSG_.joint5 = velocities[4];
                JOINT_VELOCITIES_MSG_.joint6 = velocities[5];
                JOINT_VELOCITIES_MSG_.joint7 = velocities[6];

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