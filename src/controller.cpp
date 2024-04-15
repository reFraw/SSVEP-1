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

class Controller {

    private:
        ros::NodeHandle nh_;

        ros::Publisher velocityPub_;
        ros::Publisher errorPub_;
        ros::Publisher currentPosePub_;
        ros::Subscriber waypointSub_;
        ros::Subscriber jointStateSub_;

        ros::ServiceClient initStateClient_;
        ros::ServiceServer initPoseServer_;

        std::vector<double> JOINT_STATE_{7};

        std::vector<std::string>  JOINT_NAMES_ = {
            "j2s7s300_joint_1",
            "j2s7s300_joint_2",
            "j2s7s300_joint_3",
            "j2s7s300_joint_4",
            "j2s7s300_joint_5",
            "j2s7s300_joint_6",
            "j2s7s300_joint_7"
        };

        double GAIN_;

        bool INIT_STATE_RECEIVED = false;
        bool POSE_SENDED = false;

        void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointStateMsg) {

            JOINT_STATE_ = jointStateMsg->position;

        }

        void waypointCallback(const thesis_msgs::DesiredWaypointConstPtr& waypointMsg) {

            Eigen::Vector3d desiredPosition = {
                waypointMsg->position.x,
                waypointMsg->position.y,
                waypointMsg->position.z
            };

            Eigen::Vector3d desiredLinearVelocity = {
                waypointMsg->linearVelocity.x,
                waypointMsg->linearVelocity.y,
                waypointMsg->linearVelocity.z
            };

            Eigen::Quaterniond desiredOrientation(
                waypointMsg->orientation.w,
                waypointMsg->orientation.x,
                waypointMsg->orientation.y,
                waypointMsg->orientation.z
            );

            Eigen::Vector3d desiredAngularVelocity = {
                waypointMsg->angularVelocity.x,
                waypointMsg->angularVelocity.y,
                waypointMsg->angularVelocity.z
            };

            std::vector<std::vector<double>> results = applyIK2(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, JOINT_STATE_, GAIN_);
            std::vector<double> joint_velocites = results[0];
            std::vector<double> errors = results[1];

            sensor_msgs::JointState jointMsg;
            thesis_msgs::Error errorMsg;

            errorMsg.positionErrorX = errors[0];
            errorMsg.positionErrorY = errors[1];
            errorMsg.positionErrorZ = errors[2];
            errorMsg.orientationErrorX = errors[4];
            errorMsg.orientationErrorY = errors[5];
            errorMsg.orientationErrorZ = errors[6];
            
            jointMsg.header.stamp = ros::Time::now();
            jointMsg.name = JOINT_NAMES_;
            jointMsg.velocity = joint_velocites;

            velocityPub_.publish(jointMsg);
            errorPub_.publish(errorMsg);

        }

        bool sendInitPose(ssvep1::sendInitPose::Request& req, ssvep1::sendInitPose::Response& res) {

            Eigen::Matrix4d T = directKinematics(JOINT_STATE_, JOINT_STATE_.size());

            Eigen::Quaterniond orientationQuat(T.block<3,3>(0,0));

            std::vector<double> position = {T(0,3), T(1,3), T(2,3)};
            std::vector<double> orientation = {orientationQuat.w(), orientationQuat.x(), orientationQuat.y(), orientationQuat.z()};

            res.position = position;
            res.orientation = orientation;

            ROS_INFO("Initial pose sended");
            POSE_SENDED = true;

            return true;

        }


    public:

        Controller() {

            if (nh_.hasParam("/GAIN")) {
                ros::param::get("/GAIN", GAIN_);
            }
            else {
                GAIN_ = 5;
            }

            initStateClient_ = nh_.serviceClient<ssvep1::sendInitState>("/initial_joint_state");
            ssvep1::sendInitState initSrv;

            while(!INIT_STATE_RECEIVED) {

                if(initStateClient_.call(initSrv)) {

                    JOINT_STATE_ = initSrv.response.initJointState;
                    INIT_STATE_RECEIVED = true;
                    ROS_INFO("Initial joint state received");

                }
                else {

                    ROS_WARN("Waiting for initial joint state");

                }

                ros::Rate{100}.sleep();

            }

            initPoseServer_ = nh_.advertiseService("/initial_pose", &Controller::sendInitPose, this);

            velocityPub_ = nh_.advertise<sensor_msgs::JointState>("/joint_velocities", 1);
            errorPub_ = nh_.advertise<thesis_msgs::Error>("/errors", 1);
            currentPosePub_ = nh_.advertise<geometry_msgs::Pose>("/current_pose", 1);
            jointStateSub_ = nh_.subscribe("/joint_state", 1, &Controller::jointStateCallback, this);
            waypointSub_ = nh_.subscribe("/desired_waypoint", 1, &Controller::waypointCallback, this);

            while(ros::ok()) {

                geometry_msgs::Pose currentPoseMsg;

                Eigen::Matrix4d T = directKinematics(JOINT_STATE_, JOINT_STATE_.size());
                Eigen::Quaterniond Q(T.block<3,3>(0,0));

                currentPoseMsg.position.x = T(0,3);
                currentPoseMsg.position.y = T(1,3);
                currentPoseMsg.position.z = T(2,3);
                currentPoseMsg.orientation.w = Q.w();
                currentPoseMsg.orientation.x = Q.x();
                currentPoseMsg.orientation.y = Q.y();
                currentPoseMsg.orientation.z = Q.z();

                currentPosePub_.publish(currentPoseMsg);

                ros::Rate{100}.sleep();
                ros::spinOnce();
            }

        }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "controller");

    Controller Controller;

    return 0;

}