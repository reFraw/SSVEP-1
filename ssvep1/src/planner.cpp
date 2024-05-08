#include <ros/ros.h>

#include <robotics_function/main_robotics_function.h>
#include <thesis_msgs/AngularVelocity.h>
#include <thesis_msgs/LinearVelocity.h>
#include <thesis_msgs/Position.h>
#include <thesis_msgs/Orientation.h>
#include <thesis_msgs/DesiredWaypoint.h>
#include <thesis_msgs/Error.h>
#include <thesis_msgs/FingerJoints.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <ssvep1/sendInitPose.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_driver/kinova_fingers_action.h>

#include <ssvep1/MoveAction.h>
#include <ssvep1/GrabAction.h>
#include <ssvep1/ReleaseAction.h>
#include <ssvep1/GrabAndReleaseAction.h>

#include <math.h>

using namespace Eigen;

class Planner
{
    private:

        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<ssvep1::MoveAction> moveServer_;
        actionlib::SimpleActionServer<ssvep1::GrabAction> grabServer_;
        actionlib::SimpleActionServer<ssvep1::ReleaseAction> releaseServer_;
        actionlib::SimpleActionServer<ssvep1::GrabAndReleaseAction> grabAndReleaseServer_;

        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerClient_;

        ssvep1::MoveResult MOVE_RESULT_;
        ssvep1::GrabResult GRAB_RESULT_;
        ssvep1::ReleaseResult RELEASE_RESULT_;
        ssvep1::GrabAndReleaseResult GRAB_AND_RELEASE_RESULT_;

        ros::ServiceClient initPoseClient_;

        kinova_msgs::SetFingersPositionGoal FINGER_POS_GOAL_;

        ros::Publisher waypointPublisher_;
        thesis_msgs::DesiredWaypoint WAYPOINT_MSG_;

        ros::Rate PLANNER_CLOCK{100};

        bool INIT_POSE_RECEIVED_ = false;
        bool IS_MOVING_ = false;
        bool IS_GRABBING_ = false;
        bool IS_RELEASING_ = false;
        bool IS_GRABBING_AND_RELEASING_ = false;
        bool IS_BUSY_;

        double SIMULATION_TIME_ = 0.00;
        double SAMPLING_TIME_ = 0.01;
        double DELAY_ = 0.5;
        double TIME_TO_GRAB_ = 30;
        double TIME_TO_RELEASE_ = 30;
        double APPROACH_DISTANCE_ = 0.07;
        double ZONE1_THRESHOLD_ = 0.3;
        double ZONE2_THRESHOLD_ = 0.6;
        double TOLERANCE_ = 0.01;

        Vector3d CURRENT_POSITION_;
        Quaterniond CURRENT_ORIENTATION_;

        void MoveActionCallback(const ssvep1::MoveGoalConstPtr& goal)
        {
            IS_BUSY_ = IS_MOVING_ || IS_GRABBING_ || IS_RELEASING_ || IS_GRABBING_AND_RELEASING_;

            if(IS_BUSY_)
            {
                MOVE_RESULT_.result = "SERVER IS BUSY.";
                moveServer_.setAborted(MOVE_RESULT_);
            }
            else {
                IS_MOVING_ = true;

                ROS_INFO("Acquiring the parameters for displacement.");

                Vector3d D = {
                    goal->displacement[0],
                    goal->displacement[1],
                    goal->displacement[2]
                };

                Quaterniond Q(rpy2rot(
                    goal->rpy[0],
                    goal->rpy[1],
                    goal->rpy[2]
                ));

                double time = goal->time;
                double velocity = goal->velocity;

                ROS_INFO("Manipulator in motion.");
                this->move(D, Q, time, velocity);
                ROS_INFO("Movement completed.");

                MOVE_RESULT_.result = "SUCCESS";
                moveServer_.setSucceeded(MOVE_RESULT_);

                IS_MOVING_ = false;
            }
        };

        void GrabActionCallback(const ssvep1::GrabGoalConstPtr& goal)
        {
            IS_BUSY_ = IS_MOVING_ || IS_GRABBING_ || IS_RELEASING_ || IS_GRABBING_AND_RELEASING_;

            if(IS_BUSY_)
            {
                GRAB_RESULT_.result = "SERVER IS BUSY.";
                grabServer_.setAborted(GRAB_RESULT_);
            }
            else
            {
                IS_GRABBING_ = true;

                ROS_INFO("Acquiring object position.");

                Vector3d objPosition = {
                    goal->object_position[0],
                    goal->object_position[1],
                    goal->object_position[2]
                };

                ROS_INFO("Grabbing object.");
                this->grab(objPosition);
                ROS_INFO("Object grabbed.");

                GRAB_RESULT_.result = "SUCCESS";
                grabServer_.setSucceeded(GRAB_RESULT_);

                IS_GRABBING_ = false;
            }
        };

        void ReleaseActionCallback(const ssvep1::ReleaseGoalConstPtr& goal)
        {
            IS_BUSY_ = IS_MOVING_ || IS_GRABBING_ || IS_RELEASING_ || IS_GRABBING_AND_RELEASING_;

            if(IS_BUSY_)
            {
                RELEASE_RESULT_.result = "SERVER IS BUSY.";
                releaseServer_.setAborted(RELEASE_RESULT_);
            }
            else
            {
                IS_RELEASING_ = true;

                ROS_INFO("Acquiring release position.");

                Vector3d relPosition = {
                    goal->release_position[0],
                    goal->release_position[1],
                    goal->release_position[2]
                };

                ROS_INFO("Releasing object.");
                this->grab(relPosition);
                ROS_INFO("Object released.");

                RELEASE_RESULT_.result = "SUCCESS";
                releaseServer_.setSucceeded(RELEASE_RESULT_);

                IS_RELEASING_ = false;
            }
        };

        void GrabAndReleaseCallback(const ssvep1::GrabAndReleaseGoalConstPtr& goal)
        {
            IS_BUSY_ = IS_MOVING_ || IS_GRABBING_ || IS_RELEASING_ || IS_GRABBING_AND_RELEASING_;

            if(IS_BUSY_)
            {
                GRAB_AND_RELEASE_RESULT_.result = "SERVER IS BUSY.";
                grabAndReleaseServer_.setAborted(GRAB_AND_RELEASE_RESULT_);
            }
            else
            {
                IS_GRABBING_AND_RELEASING_ = true;
                
                ROS_INFO("Acquiring object position.");

                Vector3d objPosition = {
                    goal->object_position[0],
                    goal->object_position[1],
                    goal->object_position[2]
                };

                ROS_INFO("Acquiring release position.");

                Vector3d relPosition = {
                    goal->release_position[0],
                    goal->release_position[1],
                    goal->release_position[2]
                };

                ROS_INFO("Starting movement.");
                this->grabAndRelease(objPosition, relPosition);
                ROS_INFO("Movement completed.");

                GRAB_AND_RELEASE_RESULT_.result = "SUCCESS";
                grabAndReleaseServer_.setSucceeded(GRAB_AND_RELEASE_RESULT_);

                IS_GRABBING_AND_RELEASING_ = false;
            }
        };


        void move(Vector3d displacement, Quaterniond finalOrientation, double time, double velocity)
        {
            double totalTime = time + DELAY_;
            int nPoints = totalTime / SAMPLING_TIME_;

            Vector3d startPoint = CURRENT_POSITION_;
            Vector3d finalPoint = CURRENT_POSITION_ + displacement;
            Quaterniond startOrientation = CURRENT_ORIENTATION_;

            Vector3d position;
            Vector3d linearVelocity;
            Quaterniond orientation;
            Quaterniond angularVelocity;

            Matrix3d Ri = startOrientation.toRotationMatrix();
            Matrix3d Rf = finalOrientation.toRotationMatrix();
            Matrix3d Rif = Ri.transpose()*Rf;

            AngleAxisd AA(Rif);
            Vector3d rotAxis = AA.axis();
            rotAxis.normalize();
            double rotAngle = AA.angle();

            for(int i=0; i < nPoints; i++)
            {
                std::vector<Vector3d> P = planSegmentWithVelocity(startPoint, finalPoint, time, velocity, SIMULATION_TIME_);
                std::vector<Quaterniond> Q = planOrientationWithVelocity(startOrientation, time, velocity, SIMULATION_TIME_, rotAxis, rotAngle);

                position = P[0];
                linearVelocity = P[1];
                orientation = Q[0];
                angularVelocity = Q[1];

                WAYPOINT_MSG_.position.x = position[0];
                WAYPOINT_MSG_.position.y = position[1];
                WAYPOINT_MSG_.position.z = position[2];
                WAYPOINT_MSG_.orientation.w = orientation.w();
                WAYPOINT_MSG_.orientation.x = orientation.x();
                WAYPOINT_MSG_.orientation.y = orientation.y();
                WAYPOINT_MSG_.orientation.z = orientation.z();
                WAYPOINT_MSG_.linearVelocity.x = linearVelocity[0];
                WAYPOINT_MSG_.linearVelocity.y = linearVelocity[1];
                WAYPOINT_MSG_.linearVelocity.z = linearVelocity[2];
                WAYPOINT_MSG_.angularVelocity.x = angularVelocity.x();
                WAYPOINT_MSG_.angularVelocity.y = angularVelocity.y();
                WAYPOINT_MSG_.angularVelocity.z = angularVelocity.z();

                SIMULATION_TIME_ += SAMPLING_TIME_;

                waypointPublisher_.publish(WAYPOINT_MSG_);
                PLANNER_CLOCK.sleep();
            }

            CURRENT_POSITION_ = position;
            CURRENT_ORIENTATION_ = orientation;

            SIMULATION_TIME_ = 0.00;
        };

        void grab(Vector3d objectPosition)
        {
            Vector3d startPosition = CURRENT_POSITION_;
            Quaterniond startOrientation = CURRENT_ORIENTATION_;

            double phaseOneTime = 0.2 * TIME_TO_GRAB_;
            double phaseTwoTime = 0.2 * TIME_TO_GRAB_;
            double phaseThreeTime = 0.2 * TIME_TO_GRAB_;
            double phaseFourTime = 0.2 * TIME_TO_GRAB_;
            double phaseFiveTime = 0.2 * TIME_TO_GRAB_;

            Quaterniond grabOrientation = computeGrabOrientation(objectPosition);
            Vector3d approachDisplacement = computeApproachDisplacement(objectPosition);

            Vector3d D1 = (objectPosition - approachDisplacement) - startPosition;
            Vector3d D2 = approachDisplacement;
            Vector3d D3 = {0.00, 0.00, 0.00};
            Vector3d D4 = -D2;
            Vector3d D5 = -D1;

            double phaseOneVelocity = D1.norm()/phaseOneTime;
            double phaseTwoVelocity = D2.norm()/phaseTwoTime;
            double phaseThreeVelocity = D3.norm()/phaseThreeTime;
            double phaseFourVelocity = D4.norm()/phaseFourTime;
            double phaseFiveVelocity = D1.norm()/phaseFiveTime;
            
            this->move(D1, grabOrientation, phaseOneTime + DELAY_, phaseOneVelocity); // Moving to the approach position.
            this->move(D2, grabOrientation, phaseTwoTime + DELAY_, phaseTwoVelocity); // Object approach.

            FINGER_POS_GOAL_.fingers.finger1 = 0.0;
            FINGER_POS_GOAL_.fingers.finger2 = 0.0;
            FINGER_POS_GOAL_.fingers.finger3 = 0.0;
            fingerClient_.sendGoal(FINGER_POS_GOAL_);
            fingerClient_.waitForResult();
            fingerClient_.getResult();

            FINGER_POS_GOAL_.fingers.finger1 = 5500;
            FINGER_POS_GOAL_.fingers.finger2 = 5500;
            FINGER_POS_GOAL_.fingers.finger3 = 5500;
            fingerClient_.sendGoal(FINGER_POS_GOAL_);
            fingerClient_.waitForResult();
            fingerClient_.getResult();

            this->move(D4, CURRENT_ORIENTATION_, phaseFourTime + DELAY_, phaseFourVelocity); // Return to the approach position.
            this->move(D5, startOrientation, phaseFiveTime + DELAY_, phaseFiveVelocity); // Return to initial position.
        };

        void release(Vector3d releasePosition)
        {
            Vector3d startPosition = CURRENT_POSITION_;
            Quaterniond startOrientation = CURRENT_ORIENTATION_;

            double phaseOneTime = 0.2 * TIME_TO_RELEASE_;
            double phaseTwoTime = 0.2 * TIME_TO_RELEASE_;
            double phaseThreeTime = 0.2 * TIME_TO_RELEASE_;
            double phaseFourTime = 0.2 * TIME_TO_RELEASE_;
            double phaseFiveTime = 0.2 * TIME_TO_RELEASE_;

            Quaterniond grabOrientation = computeGrabOrientation(releasePosition);
            Vector3d approachDisplacement = computeApproachDisplacement(releasePosition);

            Vector3d D1 = (releasePosition - approachDisplacement) - startPosition;
            Vector3d D2 = approachDisplacement;
            Vector3d D3 = {0.00, 0.00, 0.00};
            Vector3d D4 = -D2;
            Vector3d D5 = -D1;

            double phaseOneVelocity = D1.norm()/phaseOneTime;
            double phaseTwoVelocity = D2.norm()/phaseTwoTime;
            double phaseThreeVelocity = D3.norm()/phaseThreeTime;
            double phaseFourVelocity = D4.norm()/phaseFourTime;
            double phaseFiveVelocity = D1.norm()/phaseFiveTime;
            
            this->move(D1, grabOrientation, phaseOneTime + DELAY_, phaseOneVelocity); // Moving to the approach position.
            this->move(D2, CURRENT_ORIENTATION_, phaseTwoTime + DELAY_, phaseTwoVelocity); // Object approach.

            FINGER_POS_GOAL_.fingers.finger1 = 0.00;
            FINGER_POS_GOAL_.fingers.finger2 = 0.00;
            FINGER_POS_GOAL_.fingers.finger3 = 0.00;
            fingerClient_.sendGoal(FINGER_POS_GOAL_);
            fingerClient_.waitForResult();
            fingerClient_.getResult();

            this->move(D4, CURRENT_ORIENTATION_, phaseFourTime + DELAY_, phaseFourVelocity); // Return to the approach position.
            this->move(D5, startOrientation, phaseFiveTime + DELAY_, phaseFiveVelocity); // Return to initial position.
        };

        void grabAndRelease(Vector3d objectPosition, Vector3d releasePosition)
        {
            this->grab(objectPosition);
            this->release(releasePosition);
        };

        Quaterniond computeGrabOrientation(Vector3d objectPosition)
        {
            double xObj = objectPosition[0];
            double yObj = objectPosition[1];
            double zObj = objectPosition[2];

            Quaterniond approachingOrientation;

            if(zObj < ZONE1_THRESHOLD_)
            {
                approachingOrientation = Quaterniond(rpy2rot(0, M_PI, 0.00));
            }
            else if(zObj > ZONE1_THRESHOLD_ && zObj < ZONE2_THRESHOLD_)
            {
                double theta = std::atan2(yObj, xObj);
                approachingOrientation = Quaterniond(rpy2rot(theta, M_PI/2, 0.00));
            }
            else{
                approachingOrientation = Quaterniond(rpy2rot(0.00, 0.00, 0.00));
            }

            return approachingOrientation;
        };

        Vector3d computeApproachDisplacement(Vector3d objectPosition)
        {
            double xObj = objectPosition[0];
            double yObj = objectPosition[1];
            double zObj = objectPosition[2];

            Vector3d approachDisplacement;

            if(zObj < ZONE1_THRESHOLD_)
            {
                approachDisplacement = Vector3d(0.00, 0.00, -APPROACH_DISTANCE_);
            }
            else if(zObj > ZONE1_THRESHOLD_ && zObj < ZONE2_THRESHOLD_)
            {
                if(xObj >= 0 && yObj >= 0)
                {
                    approachDisplacement = Vector3d(APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.00);
                }
                else if(xObj < 0 && yObj >= 0)
                {
                    approachDisplacement = Vector3d(-APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.00);
                }
                else if(xObj >= 0 && yObj < 0)
                {
                    approachDisplacement = Vector3d(APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.00);
                }
                else
                {
                    approachDisplacement = Vector3d(-APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.00);
                }

                approachDisplacement /= std::sqrt(2);
            }
            else 
            {
                approachDisplacement = Vector3d(0.00, 0.00, APPROACH_DISTANCE_);
            }

            return approachDisplacement;
        };

    public:

        Planner() :
            moveServer_(nh_, "Move", boost::bind(&Planner::MoveActionCallback, this, _1), false),
            grabServer_(nh_, "Grab", boost::bind(&Planner::GrabActionCallback, this, _1), false),
            releaseServer_(nh_, "Release", boost::bind(&Planner::ReleaseActionCallback, this, _1), false),
            grabAndReleaseServer_(nh_, "GrabRel", boost::bind(&Planner::GrabAndReleaseCallback, this, _1), false),
            fingerClient_("finger", true)
        {
            ros::param::get("/SAMPLING_TIME", SAMPLING_TIME_);
            ros::param::get("/TIME_DELAY", DELAY_);
            ros::param::get("/TIME_TO_GRAB", TIME_TO_GRAB_);
            ros::param::get("/TIME_TO_RELEASE", TIME_TO_RELEASE_);
            ros::param::get("/APPROACH_DISTANCE", APPROACH_DISTANCE_);
            ros::param::get("/ZONE1_THRESHOLD", ZONE1_THRESHOLD_);
            ros::param::get("/ZONE2_THRESHOLD", ZONE2_THRESHOLD_);

            initPoseClient_ = nh_.serviceClient<ssvep1::sendInitPose>("/initial_pose");
            ssvep1::sendInitPose initPoseSrv;

            while(!INIT_POSE_RECEIVED_)
            {
                if(initPoseClient_.call(initPoseSrv))
                {
                    CURRENT_POSITION_ = {
                        initPoseSrv.response.position[0],
                        initPoseSrv.response.position[1],
                        initPoseSrv.response.position[2],
                    };

                    CURRENT_ORIENTATION_ = Quaterniond(
                        initPoseSrv.response.orientation[0],
                        initPoseSrv.response.orientation[1],
                        initPoseSrv.response.orientation[2],
                        initPoseSrv.response.orientation[3]
                    );

                    ROS_INFO("Initial manipulator pose received.");
                    INIT_POSE_RECEIVED_ = true;
                }
                else
                {
                    ROS_WARN("Waiting for initial manipulator pose.");
                }

                PLANNER_CLOCK.sleep();
            }
            initPoseClient_.shutdown();

            waypointPublisher_ = nh_.advertise<thesis_msgs::DesiredWaypoint>("/desired_waypoint", 1);

            moveServer_.start();
            grabServer_.start();
            releaseServer_.start();
            grabAndReleaseServer_.start();

            ros::spin();
        };

        ~Planner(){};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");

    Planner Planner;

    return 0;
}