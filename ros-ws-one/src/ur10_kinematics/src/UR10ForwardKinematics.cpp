#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"

#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"

#include "sstream"
#include "iostream"
#include "fstream"

#define UR10_DOF 6

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
    std::cout << "Calculated Position   :" << "[ "<< calculatedPosition.position.x << " , " << calculatedPosition.position.y << " , " << calculatedPosition.position.z << " ]" << std::endl;
    std::cout << "End Effector Position :" << "[ "<< endEffectorPositionMessage->pose.position.x << " , " << endEffectorPositionMessage->pose.position.y << " , " << endEffectorPositionMessage->pose.position.z << " ]" << std::endl;
    std::cout << "Forward Kinematic Error :" << calculateError(*endEffectorPositionMessage) << std::endl;
}

KDL::Chain initChainUR10() {
    KDL::Chain chain;

    //TODO You should write the correct chain for UR10

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0,	0,	0	))));

    return chain;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"UR10_Forward_Kinematics");

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

    ros::Rate loop(10);

    KDL::Chain chain = initChainUR10();

    float jointValues[7] = {0};

    while(ros::ok()) {
        /*
        * TODO: You should read values of joints[0:5] from file as degree.
        */

       /*
        * TODO: You should calculate the End Effector Position from ChainFkSolverPos_recursive (Not: This library uses radian (joint * PI / 180)).
        */


       /*
        * TODO: Publish the found joint angels to topic.
        */

       /*
        * TODO: Fill the calculatedLocation variable for error calculation.
        */

        ros::spinOnce();
        loop.sleep();
    }


	return 0;
}
