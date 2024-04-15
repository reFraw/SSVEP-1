#include <ros/ros.h>

#include <robotics_function/main_robotics_function.h>
#include <thesis_msgs/AngularVelocity.h>
#include <thesis_msgs/LinearVelocity.h>
#include <thesis_msgs/Position.h>
#include <thesis_msgs/Orientation.h>
#include <thesis_msgs/DesiredWaypoint.h>
#include <thesis_msgs/Error.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <ssvep1/sendInitPose.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

class Planner {

    private:

        ros::NodeHandle nh_;
        
        ros::ServiceClient initPoseClient_;
        
        ros::Publisher waypointPub_;
        ros::Publisher readyPub_;
        ros::Subscriber instrSub_;

        ros::Rate loopRate_{100};

        bool INIT_STATE_RECEIVED_ = false;

        double SIMULATION_TIME_ = 0.00;

        double SAMPLING_TIME_ = 0.01;
        double DELAY_ = 0.5;
        double TIME_TO_GRAB_ = 20;
        double TIME_TO_RELEASE_ = 20;
        double APPROACH_DISTANCE_ = 0.07;
        double ZONE1_THRESHOLD_ = 0.3;
        double ZONE2_THRESHOLD_ = 0.6;

        Vector3d CURRENT_POSITION_;
        Quaterniond CURRENT_QUATERNION_;

        Vector3d APPROACH_ZONE_1_3_ = {0.0, 0.0, APPROACH_DISTANCE_};
        Quaterniond APPROACH_ORIENTATION_1_{rpy2rot(0, 3.1415, 0)};
        Quaterniond APPROACH_ORIENTATION_3_{rpy2rot(0.0, 0.0, 0.0)};

        std::string INSTRUCTION_TYPE_ = "STAY";
        std::vector<std::string> INSTR_COMP;

        thesis_msgs::DesiredWaypoint WAYPOINT_MSG_;
        std_msgs::Bool READY_MSG_;




        std::vector<std::string> splitString(std::string input) {

            std::vector<std::string> result;
            std::istringstream iss(input);
            std::string segment;

            while (std::getline(iss, segment, '|')) {
                std::istringstream segmentStream(segment);
                std::string element;

                while (std::getline(segmentStream, element, ',')) {
                    result.push_back(element);
                }
            }

            return result;

        }  

        Quaterniond computeGrabOrientation(Vector3d objectPosition) {

            double theta = std::atan2(objectPosition[1], objectPosition[0]);

            return Quaterniond(rpy2rot(theta, 3.1415/2, 0));

        }


        void instructionCallback(const std_msgs::StringConstPtr& instrMsg) {

            std::string instrString = instrMsg->data;
            std::vector<std::string> specifies = splitString(instrString);

            INSTR_COMP.resize(specifies.size());
            INSTR_COMP = specifies;

            INSTRUCTION_TYPE_ = specifies[0];

        }

        void stay() {

            READY_MSG_.data = true;

            WAYPOINT_MSG_.position.x = CURRENT_POSITION_[0];
            WAYPOINT_MSG_.position.y = CURRENT_POSITION_[1];
            WAYPOINT_MSG_.position.z = CURRENT_POSITION_[2];
            WAYPOINT_MSG_.orientation.w = CURRENT_QUATERNION_.w();
            WAYPOINT_MSG_.orientation.x = CURRENT_QUATERNION_.x();
            WAYPOINT_MSG_.orientation.y = CURRENT_QUATERNION_.y();
            WAYPOINT_MSG_.orientation.z = CURRENT_QUATERNION_.z();
            WAYPOINT_MSG_.linearVelocity.x = 0.0;
            WAYPOINT_MSG_.linearVelocity.y = 0.0;
            WAYPOINT_MSG_.linearVelocity.z = 0.0;
            WAYPOINT_MSG_.angularVelocity.x = 0.0;
            WAYPOINT_MSG_.angularVelocity.y = 0.0;
            WAYPOINT_MSG_.angularVelocity.z = 0.0;

            waypointPub_.publish(WAYPOINT_MSG_);
            readyPub_.publish(READY_MSG_);

            loopRate_.sleep();

        };

        void move(Vector3d d, Quaterniond finalOrientation, double finalTime, double cruiseVelocity) {

            double totalTime = finalTime + DELAY_;
            int nPoints = totalTime / SAMPLING_TIME_;

            Vector3d finalPosition = CURRENT_POSITION_ + d;

            Vector3d position;
            Quaterniond orientation;
            Vector3d linearVelocity;
            Quaterniond angularVelocity;

            for(int i = 0; i < nPoints; i++) {

                std::vector<Vector3d> P = planSegmentWithVelocity(CURRENT_POSITION_, finalPosition, totalTime, cruiseVelocity, SIMULATION_TIME_);
                std::vector<Quaterniond> Q = planOrientationWithVelocity(CURRENT_QUATERNION_, finalOrientation, totalTime, cruiseVelocity, SIMULATION_TIME_);

                position = P[0];
                linearVelocity = P[1];
                orientation = Q[0];
                angularVelocity = Q[1];

                READY_MSG_.data = false;

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

                waypointPub_.publish(WAYPOINT_MSG_);
                readyPub_.publish(READY_MSG_);

                SIMULATION_TIME_ += SAMPLING_TIME_;

                loopRate_.sleep();

            }

            CURRENT_POSITION_ = position;
            CURRENT_QUATERNION_ = orientation;

            SIMULATION_TIME_ = 0.00;

        };

        void grab(Vector3d objectPosition) {

            Vector3d startPosition = CURRENT_POSITION_;
            Quaterniond startOrientation = CURRENT_QUATERNION_;

            double phaseOneTime = 0.2 * TIME_TO_GRAB_;      // Move to approach point
            double phaseTwoTime = 0.2 * TIME_TO_GRAB_;      // Approach
            double phaseThreeTime = 0.2 * TIME_TO_GRAB_;    // Grab object
            double phaseFourTime = 0.2 * TIME_TO_GRAB_;     // Return to apprach point
            double phaseFiveTime = 0.2 * TIME_TO_GRAB_;     // Return to start position

            Vector3d d1;
            Vector3d d2;
            Vector3d d3;
            Vector3d d4;
            Vector3d d5;

            Quaterniond approachOrientation;

            double xObj = objectPosition[0];
            double yObj = objectPosition[1];
            double elevation = objectPosition[2];

            if (elevation < ZONE1_THRESHOLD_) {

                d1 = (objectPosition + APPROACH_ZONE_1_3_) - startPosition;
                d2 = -APPROACH_ZONE_1_3_;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per presa oggetto
                d4 = -d2;
                d5 = -d1;

                approachOrientation = APPROACH_ORIENTATION_1_;

            }
            else if (elevation >= ZONE1_THRESHOLD_ && elevation < ZONE2_THRESHOLD_) {

                approachOrientation = this->computeGrabOrientation(objectPosition);

                Vector3d APPROACH_ZONE_2;

                if(xObj >= 0 && yObj >= 0) {
                    APPROACH_ZONE_2 = {APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj < 0 && yObj >= 0) {
                    APPROACH_ZONE_2 = {-APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj >= 0 && yObj < 0) {
                    APPROACH_ZONE_2 = {APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj < 0 && yObj < 0) {
                    APPROACH_ZONE_2 = {-APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.0};

                }

                APPROACH_ZONE_2 /= std::sqrt(2);

                d1 = (objectPosition - APPROACH_ZONE_2) - startPosition;
                d2 = APPROACH_ZONE_2;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per presa oggetto
                d4 = -d2;
                d5 = -d1;

            }
            else if (elevation >= ZONE2_THRESHOLD_) {

                d1 = (objectPosition - APPROACH_ZONE_1_3_) - startPosition;
                d2 = APPROACH_ZONE_1_3_;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per presa oggetto
                d4 = -d2;
                d5 = -d1;

                approachOrientation = APPROACH_ORIENTATION_3_;

            }

            double phaseOneVelocity = d1.norm() / phaseOneTime + DELAY_;
            double phaseTwoVelocity = d2.norm() / phaseTwoTime + DELAY_;
            double phaseThreeVelocity = d3.norm() / phaseThreeTime + DELAY_;
            double phaseFourVelocity = d4.norm() / phaseFourTime + DELAY_;
            double phaseFiveVelocity = d5.norm() / phaseFiveTime + DELAY_;

            this->move(d1, approachOrientation, phaseOneTime + DELAY_, phaseOneVelocity);
            this->move(d2, CURRENT_QUATERNION_, phaseTwoTime + DELAY_, phaseTwoVelocity);


            //TODO Inserire primitiva per presa oggetto
            this->move(d3, CURRENT_QUATERNION_, phaseThreeTime + DELAY_, phaseThreeTime);
            
            this->move(d4, CURRENT_QUATERNION_, phaseFourTime + DELAY_, phaseFourVelocity);
            this->move(d5, startOrientation, phaseFiveTime + DELAY_, phaseFiveVelocity);

        };

        void release(Vector3d objectPosition) {

            Vector3d startPosition = CURRENT_POSITION_;
            Quaterniond startOrientation = CURRENT_QUATERNION_;

            double phaseOneTime = 0.2 * TIME_TO_RELEASE_;      // Move to approach point
            double phaseTwoTime = 0.2 * TIME_TO_RELEASE_;      // Approach
            double phaseThreeTime = 0.2 * TIME_TO_RELEASE_;    // release object
            double phaseFourTime = 0.2 * TIME_TO_RELEASE_;     // Return to apprach point
            double phaseFiveTime = 0.2 * TIME_TO_RELEASE_;     // Return to start position

            Vector3d d1;
            Vector3d d2;
            Vector3d d3;
            Vector3d d4;
            Vector3d d5;

            Quaterniond approachOrientation;

            double xObj = objectPosition[0];
            double yObj = objectPosition[1];
            double elevation = objectPosition[2];

            if (elevation < ZONE1_THRESHOLD_) {

                d1 = (objectPosition + APPROACH_ZONE_1_3_) - startPosition;
                d2 = -APPROACH_ZONE_1_3_;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per rilascio oggetto
                d4 = -d2;
                d5 = -d1;

                approachOrientation = APPROACH_ORIENTATION_1_;

            }
            else if (elevation >= ZONE1_THRESHOLD_ && elevation < ZONE2_THRESHOLD_) {

                approachOrientation = this->computeGrabOrientation(objectPosition);

                Vector3d APPROACH_ZONE_2;

                if(xObj >= 0 && yObj >= 0) {
                    APPROACH_ZONE_2 = {APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj < 0 && yObj >= 0) {
                    APPROACH_ZONE_2 = {-APPROACH_DISTANCE_, APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj >= 0 && yObj < 0) {
                    APPROACH_ZONE_2 = {APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.0};

                }
                else if(xObj < 0 && yObj < 0) {
                    APPROACH_ZONE_2 = {-APPROACH_DISTANCE_, -APPROACH_DISTANCE_, 0.0};

                }

                d1 = (objectPosition - APPROACH_ZONE_2) - startPosition;
                d2 = APPROACH_ZONE_2;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per rilascio oggetto
                d4 = -d2;
                d5 = -d1;

            }
            else if (elevation >= ZONE2_THRESHOLD_) {

                d1 = (objectPosition - APPROACH_ZONE_1_3_) - startPosition;
                d2 = APPROACH_ZONE_1_3_;
                d3 = {0.00, 0.00, 0.00}; // TODO Inserire routine per rilascio oggetto
                d4 = -d2;
                d5 = -d1;

                approachOrientation = APPROACH_ORIENTATION_3_;

            }

            double phaseOneVelocity = d1.norm() / phaseOneTime + DELAY_;
            double phaseTwoVelocity = d2.norm() / phaseTwoTime + DELAY_;
            double phaseThreeVelocity = d3.norm() / phaseThreeTime + DELAY_;
            double phaseFourVelocity = d4.norm() / phaseFourTime + DELAY_;
            double phaseFiveVelocity = d5.norm() / phaseFiveTime + DELAY_;

            this->move(d1, approachOrientation, phaseOneTime + DELAY_, phaseOneVelocity);
            this->move(d2, CURRENT_QUATERNION_, phaseTwoTime + DELAY_, phaseTwoVelocity);

            //TODO Inserire primitiva per rilascio oggetto
            this->move(d3, CURRENT_QUATERNION_, phaseThreeTime + DELAY_, phaseThreeTime);

            this->move(d4, CURRENT_QUATERNION_, phaseFourTime + DELAY_, phaseFourVelocity);
            this->move(d5, startOrientation, phaseFiveTime + DELAY_, phaseFiveVelocity);

        };

        void moveObject(Vector3d objectPosition, Vector3d releasePosition) {

            this->grab(objectPosition);
            this->release(releasePosition);

        };


    public:

        Planner() {

            ros::param::get("/SAMPLING_TIME", SAMPLING_TIME_);
            ros::param::get("/TIME_DELAY", DELAY_);
            ros::param::get("/TIME_TO_GRAB", TIME_TO_GRAB_);
            ros::param::get("/TIME_TO_RELEASE", TIME_TO_RELEASE_);
            ros::param::get("/APPROACH_DISTANCE", APPROACH_DISTANCE_);
            ros::param::get("/ZONE1_THRESHOLD", ZONE1_THRESHOLD_);
            ros::param::get("/ZONE2_THRESHOLD", ZONE2_THRESHOLD_);

            initPoseClient_ = nh_.serviceClient<ssvep1::sendInitPose>("/initial_pose");
            ssvep1::sendInitPose initPoseSrv;

            while(!INIT_STATE_RECEIVED_) {

                if(initPoseClient_.call(initPoseSrv)) {
                    
                    std::vector<double> respPosition = initPoseSrv.response.position;
                    std::vector<double> respQuaterniond = initPoseSrv.response.orientation;

                    CURRENT_POSITION_ = {respPosition[0], respPosition[1], respPosition[2]};
                    CURRENT_QUATERNION_ = Quaterniond{respQuaterniond[0], respQuaterniond[1], respQuaterniond[2], respQuaterniond[3]};

                    INIT_STATE_RECEIVED_ = true;
                    ROS_INFO("Initial pose received");
                }
                else {
                    ROS_WARN("Waiting for initial pose");
                }

                loopRate_.sleep();

            }

            initPoseClient_.shutdown();

            waypointPub_ = nh_.advertise<thesis_msgs::DesiredWaypoint>("/desired_waypoint", 1);
            readyPub_ = nh_.advertise<std_msgs::Bool>("/ready", 1);
            instrSub_ = nh_.subscribe("/instructions", 1, &Planner::instructionCallback, this);

            while(ros::ok())  {
                
                if (INSTRUCTION_TYPE_ == "STAY") {

                    this->stay();

                }
                else if (INSTRUCTION_TYPE_ == "MOVE") {

                    Vector3d d = {
                        std::stod(INSTR_COMP[1]),
                        std::stod(INSTR_COMP[2]),
                        std::stod(INSTR_COMP[3])
                    };

                    Quaterniond finalOrientation(rpy2rot(
                        std::stod(INSTR_COMP[4]),
                        std::stod(INSTR_COMP[5]),
                        std::stod(INSTR_COMP[6])
                    ));

                    double time = std::stod(INSTR_COMP[7]);
                    double velocity = std::stod(INSTR_COMP[8]);

                    this->move(d, finalOrientation, time, velocity);

                    INSTRUCTION_TYPE_ = "STAY";

                }
                else if (INSTRUCTION_TYPE_ == "GRAB") {

                    Vector3d objectPosition = {
                        std::stod(INSTR_COMP[1]),
                        std::stod(INSTR_COMP[2]),
                        std::stod(INSTR_COMP[3])
                    };

                    this->grab(objectPosition);

                    INSTRUCTION_TYPE_ = "STAY";

                }
                else if (INSTRUCTION_TYPE_ == "RELEASE") {

                    Vector3d objectPosition = {
                        std::stod(INSTR_COMP[1]),
                        std::stod(INSTR_COMP[2]),
                        std::stod(INSTR_COMP[3])
                    };

                    this->release(objectPosition);

                    INSTRUCTION_TYPE_ = "STAY";

                }
                else if (INSTRUCTION_TYPE_ == "GRABREL") {

                    Vector3d objectPosition = {
                        std::stod(INSTR_COMP[1]),
                        std::stod(INSTR_COMP[2]),
                        std::stod(INSTR_COMP[3])
                    };

                    Vector3d releasePosition = {
                        std::stod(INSTR_COMP[4]),
                        std::stod(INSTR_COMP[5]),
                        std::stod(INSTR_COMP[6])
                    };

                    this->moveObject(objectPosition, releasePosition);

                    INSTRUCTION_TYPE_ = "STAY";

                }

                ros::spinOnce();

            }
        }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "planner");

    Planner Planner;

    return 0;

}