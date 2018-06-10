#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"

#define UR10_DOF 6
#define PI 3.1415926535897932384626433f

geometry_msgs::PoseStamped endEffectorPosition;

geometry_msgs::PoseArray cupPositions;
bool isCupPositionsAvailable = false;

geometry_msgs::PoseArray cupTargetPositions;
bool isCupTargetPositionsAvailable = false;

uint8_t cupIndex = 0;
std::vector<bool> isCupReached;
std::vector<bool> isCupGripped;
std::vector<bool> isCupTargetReached;
std::vector<bool> isCupFinished;

geometry_msgs::Pose calculatedPosition;

float calculateError (const geometry_msgs::PoseStamped& endEffectorPosition) {
    float error = 0;
    error += pow(calculatedPosition.position.x - endEffectorPosition.pose.position.x,2) * 10000;
    error += pow(calculatedPosition.position.y - endEffectorPosition.pose.position.y,2) * 10000;
    error += pow(calculatedPosition.position.z - endEffectorPosition.pose.position.z,2) * 10000;
    error = sqrt(error);
    return error;
}

/*
 * TODO: You should show the error on the console.
 */
void endEffectorPositionCallback(const geometry_msgs::PoseStampedConstPtr& endEffectorPositionMessage) {
    endEffectorPosition = *endEffectorPositionMessage;
    std::cout << "Calculated Position   :" << "[ "<< calculatedPosition.position.x << " , " << calculatedPosition.position.y << " , " << calculatedPosition.position.z << " ]" << std::endl;
    std::cout << "End Effector Position :" << "[ "<< endEffectorPositionMessage->pose.position.x << " , " << endEffectorPositionMessage->pose.position.y << " , " << endEffectorPositionMessage->pose.position.z << " ]" << std::endl;
    std::cout << "Bonus Kinematic Error :" << calculateError(*endEffectorPositionMessage) << std::endl;
}

void cupPositionaCallback(const geometry_msgs::PoseArrayConstPtr& cupPositionsCallback) {
    cupPositions = *cupPositionsCallback.get();
    isCupPositionsAvailable = true;
}

void cupTargetPositionaCallback(const geometry_msgs::PoseArrayConstPtr& cupTargetPositionsCallback) {
    cupTargetPositions = *cupTargetPositionsCallback.get();
    isCupTargetPositionsAvailable = true;
}

KDL::Chain initChainUR10() {
    KDL::Chain chain;

    //TODO You should write the correct chain for UR10

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0,	0,	0	))));

    return chain;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"UR10_Bonus_Kinematics");

    ros::NodeHandle node;

    ros::Publisher joints[6];
    joints[0] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint001",1000);
    joints[1] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint002",1000);
    joints[2] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint003",1000);
    joints[3] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint004",1000);
    joints[4] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint005",1000);
    joints[5] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint006",1000);

    for(int index=0;index<UR10_DOF;index++){
        std_msgs::Float32 jointMessage;
        jointMessage.data = 0;
        joints[index].publish(jointMessage);
    }

    ros::Subscriber endEffectorPositionSubscriber = node.subscribe<geometry_msgs::PoseStamped>("/UR10/positions/endEffectorPosition",1000,endEffectorPositionCallback);

    ros::Subscriber cupPositionsSubscriber = node.subscribe<geometry_msgs::PoseArray>("/task/positions/cups",1000,cupPositionaCallback);
    ros::Subscriber cupTargetPositionsSubscriber = node.subscribe<geometry_msgs::PoseArray>("/task/positions/cuptargets",1000,cupTargetPositionaCallback);

    ros::Publisher gripper = node.advertise<std_msgs::Float32>("/UR10/gripper/closingAngle",1000);

    ros::Rate loop(10);

    KDL::Chain chain = initChainUR10();

    while(ros::ok()) {
        if(isCupPositionsAvailable && isCupTargetPositionsAvailable) {
            /*
             * TODO: Take the cup and give it to Bill
             */
        }

        ros::spinOnce();
        loop.sleep();
    }


	return 0;
}
