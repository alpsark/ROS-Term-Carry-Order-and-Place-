#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolvervel_pinv_givens.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/frames_io.hpp"

#include <vector>

#define UR10_DOF 6

geometry_msgs::PoseArray targetPositions;
bool isTargetPositionsAvailable = false;

geometry_msgs::Pose calculatedPosition;

float currentTarget = -1;
bool isCurrentTargetAvailable = false;

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
    std::cout << "Calculated Position   :" << "[ "<< calculatedPosition.position.x << " , " << calculatedPosition.position.y << " , " << calculatedPosition.position.z << " ]" << std::endl;
    std::cout << "End Effector Position :" << "[ "<< endEffectorPositionMessage->pose.position.x << " , " << endEffectorPositionMessage->pose.position.y << " , " << endEffectorPositionMessage->pose.position.z << " ]" << std::endl;
    std::cout << "Inverse Kinematic Error :" << calculateError(*endEffectorPositionMessage) << std::endl;
}

void currentTargetCallback(const std_msgs::Float32ConstPtr& currentTargetMessage) {
    currentTarget = currentTargetMessage->data;
    isCurrentTargetAvailable = true;
}

void targetPositionaCallback(const geometry_msgs::PoseArrayConstPtr& targetPositionsCallback) {
    targetPositions = *targetPositionsCallback.get();
    isTargetPositionsAvailable = true;
}

KDL::Chain initChainUR10() {
    KDL::Chain chain;

    //TODO You should write the correct chain for UR10

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0,	0,	0	))));

    return chain;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"UR10_Inverse_Kinematics");

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

    ros::Subscriber targetPositionsSubscriber = node.subscribe<geometry_msgs::PoseArray>("/task/positions/targets",1000,targetPositionaCallback);

    //You can use rostopic pub /Baxter/targets/currentTarget std_msgs/Float32 1 for  changing currentTarget to 1
    ros::Subscriber currentTargetSubscriber = node.subscribe<std_msgs::Float32>("/task/targets/currentTarget",1000,currentTargetCallback);

    ros::Rate loop(10);

    KDL::Chain chain = initChainUR10();

    while(ros::ok()) {
        KDL::ChainFkSolverPos_recursive forwardKinematicSolver = KDL::ChainFkSolverPos_recursive(chain);

        KDL::Frame goalEndEffectorPosition;

        bool isCurrentSetted = false;

        if(isCurrentTargetAvailable && isTargetPositionsAvailable) {
            if(currentTarget >= 0 && currentTarget < targetPositions.poses.size()) {
                calculatedPosition = targetPositions.poses.at(currentTarget);
                isCurrentSetted = true;
            }
        }

        if(isCurrentSetted) {
            goalEndEffectorPosition = KDL::Frame(
                        KDL::Rotation::RPY(0,0,0),
                        KDL::Vector(calculatedPosition.position.x,calculatedPosition.position.y,calculatedPosition.position.z)
            );

            /*
             * TODO: Calculate the Joint angles for target positons. You can use ChainFkSolverPos_recursive and ChainIkSolverPos_LMA.
             */

            // You can use this matrix for ChainIkSolverPos_LMA
            Eigen::Matrix<double,6,1> L;
            L(0)=1;L(1)=1;L(2)=1;
            L(3)=0.01;L(4)=0.01;L(5)=0.01;


            /*
             * TODO: Publish the found joint angles.
             */

            /*
             * TODO: Fill the calculatedLocation variable for error calculation. It is same as targetpoint.
             */
        }

        ros::spinOnce();
        loop.sleep();
    }


    return 0;
}
