/*
 * my_subsumption.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: okan
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <vector>

#include "map_helper.h"
#include "image_helper.h"

#define PI 3.1415926535897932384626433f
/**
 * The node which runs simple subsumption architecture
 * @param argc
 * @param argv
 * @return
 */

//0.5 safe


std::string prev_behavior;
ros::Publisher scanEnable ;
ros::Publisher beaconEnable ;
ros::Publisher CurrentBehavior ;
MapHelper mapHelper;
ImageHelper imageHelper;
float robotX;
float robotY;
float robotYaw;
float redx;
float redy;
float greenx;
float greeny;
bool took_order;
bool isredgotoposecompleted;
bool isgreengotoposecompleted;
bool ispickedupobject;

bool go_near_beacon(){
		// false ;
	if(!isredgotoposecompleted){
		return true ;
	}else{return false;}
}
bool order(){
		//return false ;
	if(!took_order){
		return true ;
	}else{return false;}
}	
bool deliver_order(){
		//return false ;
	if(!isgreengotoposecompleted){
		return true ;
	}else{return false;}
}
bool pick_up_order(){
	if(!ispickedupobject){
		return true ;
	}else{
	took_order = false;
	isgreengotoposecompleted = false;
	isredgotoposecompleted = false;
	ispickedupobject = false;
	return false;}
}

std_msgs::String Choosebehavior(){//choose next beahvior
	std_msgs::Bool data;
	data.data = true;
		//beaconEnable.publish(data);
	std_msgs::String behaviour;
	if(go_near_beacon()){
		if(prev_behavior != "go_near_beacon"){
               std::cout << "ChoosenBehavior is go_near_red_beacon" <<std::endl;
		}			
		scanEnable.publish(data);	
		data.data = false;	
		beaconEnable.publish(data);
		behaviour.data =  "go_near_beacon" ;
		prev_behavior = "go_near_beacon";
		return  behaviour ;
	}
	if(order()){
		if(prev_behavior != "order"){
		std::cout << "ChoosenBehavior is order" <<std::endl;
		}		
		data.data = false;	
		scanEnable.publish(data);
		beaconEnable.publish(data);
		behaviour.data =  "order" ;
		prev_behavior = "order";
		return  behaviour ;
	}
	if(deliver_order()){
		scanEnable.publish(data);
		data.data = false;	
		beaconEnable.publish(data);
		if(prev_behavior != "deliver_order"){
		std::cout << "ChoosenBehavior is deliver_order" <<std::endl;
		}			
		behaviour.data =  "deliver_order" ;
		prev_behavior = "deliver_order";
		return  behaviour ;
	}
	if(pick_up_order()){
		if(prev_behavior != "pick_up_order"){
		data.data = false;	
		scanEnable.publish(data);
		data.data = true;	
		beaconEnable.publish(data);
		std::cout << "ChoosenBehavior is pick_up_order" <<std::endl;	
		}		
		behaviour.data =  "pick_up_order" ;
		prev_behavior = "pick_up_order";
		return  behaviour ;
	}
} 
void updatePose(const geometry_msgs::PoseWithCovarianceStamped& NewPose){
	geometry_msgs::PoseWithCovariance  RobotPosewithcovariance = NewPose.pose;
	geometry_msgs::Pose  RobotPose = RobotPosewithcovariance.pose;
	robotX=RobotPose.position.x;
	robotY=RobotPose.position.y;
	robotYaw=tf::getYaw(RobotPose.orientation);
	//std::cout <<prefix << "PoseUpdated: " << robotX << "," << robotY << "," << robotYaw <<std::endl;
	
}

//collects sensor data and decides behavior
//avoid_object
//explore
//go_near_Beacon
//get_order
//deliver_order
//pick_up_order

void findBeaconLocation(float th,std::string color){
    float iteration = 0.1 ;
    float i = 0;
    while(true){
       if( mapHelper.isReallyOccupied(robotX+ i*cos(robotYaw-th) , robotY+ i*sin(robotYaw-th) )) {
          if(color == "green"){
            greenx = robotX+ (i)*cos(robotYaw-th);
            greeny = robotY+ (i)*sin(robotYaw-th);
          }else if(color == "red"){
            redx = robotX + (i)*cos(robotYaw-th);
            redy = robotY+ (i)*sin(robotYaw-th);
            
          }
          return;
        }
        i += iteration ; 
    }

}

void took_func (const std_msgs::Bool::ConstPtr& BoolPointer ){
    if(BoolPointer->data ){
        took_order = true;
    }else{
        took_order = false;
    }
}

void Red_gotoPoseCompleted(const std_msgs::Bool::ConstPtr& BoolPointer ){
    if(BoolPointer->data ){
        isredgotoposecompleted = true;
    }else{
        isredgotoposecompleted = false;
    }
}

void Green_gotoPoseCompleted(const std_msgs::Bool::ConstPtr& BoolPointer ){
    if(BoolPointer->data ){
        isgreengotoposecompleted = true;
    }else{
        isgreengotoposecompleted = false;
    }
}

void pickedupobject_func(const std_msgs::Bool::ConstPtr& BoolPointer ){
    if(BoolPointer->data ){
        ispickedupobject = true;
    }else{
        ispickedupobject = false;
    }
}

int main(int argc, char** argv)
{
	
	prev_behavior = "no behavior";
	ros::init(argc,argv,"my_subsumption" );
	ros::NodeHandle nh;
	ros::Rate rate(30);
		
	// construct publishers and subscribers
	scanEnable = nh.advertise<std_msgs::Bool>("/OmniPlatform001/sensors/hokuyoEnable", 1);
    	CurrentBehavior = nh.advertise<std_msgs::String>( "/curbehavior", 1);
	//ros::Publisher took_orderPub = nh.advertise<std_msgs::Bool>("/took_order", 1);
	beaconEnable = nh.advertise<std_msgs::Bool>("/tools/sensors/greenBeaconEnable", 1);


	//ros::Subscriber laserSubs = nh.subscribe("/OmniPlatform001/sensors/scan", 10,  processLaserScan);
	ros::Subscriber mapSubs = nh.subscribe("/map", 10, &MapHelper::processMap, &mapHelper);
	ros::Subscriber poseSubs = nh.subscribe("/amcl_pose", 10, updatePose); //geometry_msgs/PoseWithCovarianceStamped
    //ros::Subscriber poseSubs = nh.subscribe("/amcl_pose", 10, updatePose); //geometry_msgs
    ros::Subscriber took_orderSubs =nh.subscribe("/took_order",1,took_func);
    ros::Subscriber RedcompleteSubs =nh.subscribe("/Red_gotoPoseCompleted",1,Red_gotoPoseCompleted);
    ros::Subscriber GreencompleteSubs =nh.subscribe("/Green_gotoPoseCompleted",1,Green_gotoPoseCompleted);
    ros::Subscriber PickedupSubs =nh.subscribe("/Pickedupobject",1,pickedupobject_func);
        
	std_msgs::Bool data;
	data.data = true; 
	scanEnable.publish(data);
	data.data = false; 
	beaconEnable.publish(data);
    took_order =false;
    isredgotoposecompleted = false;
    isgreengotoposecompleted = false;
    ispickedupobject = false;
	while(ros::ok())
	{
		
        CurrentBehavior.publish(Choosebehavior());
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
