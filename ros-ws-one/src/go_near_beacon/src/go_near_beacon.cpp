#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "map_helper.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <vector>

#include "MyRRTSolver.h"
#include "graph/MyVertex.h"
#include "graph/MyGraph.h"
#include "graph/MyEdge.h"
#include "image_helper.h"

#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.52359879
#define MIN_TURN_SPD 0.05

#define MAX_LINEAR_SPD 0.7
#define EPS 0.05

bool isbeavioractive ;
bool gotoPoseCompleted ;
bool robotposeGet ;
bool obstacle_detected;
bool isrobot_scan_all_area;
ros::Publisher	completedPub;
float robotX;
float robotY;
float robotYaw;
std::string prefix;

bool behavior_changed;

void updatePose(const geometry_msgs::PoseWithCovarianceStamped& NewPose){
	geometry_msgs::PoseWithCovariance  RobotPosewithcovariance = NewPose.pose;
	geometry_msgs::Pose  RobotPose = RobotPosewithcovariance.pose;
	robotX=RobotPose.position.x;
	robotY=RobotPose.position.y;
	robotYaw=tf::getYaw(RobotPose.orientation);	
        robotposeGet = true ;
}

float normalizeRad(float rad)
{
    while(rad>PI)
    {
        rad=rad-2*PI;
    }
    while(rad<-PI)
    {
        rad=rad+2*PI;
    }
    return rad;
}

geometry_msgs::Twist gotoPose(float targetPosex , float targetPosey , float targetYaw){
    // create velocity vector
	geometry_msgs::Twist moveCmd;
	moveCmd.linear.x=0;
	moveCmd.linear.y=0;
	moveCmd.angular.z=0;

	float diffx = targetPosex - robotX;
	float diffy = targetPosey - robotY;
	float yawDiff = normalizeRad(targetYaw - robotYaw);
    int sign = yawDiff < 0?-1:1;
    
    // check completed
    
	if (fabs(diffx) < EPS && fabs(diffy) < EPS) {

		if(fabs(yawDiff) < 2 * EPS){
			gotoPoseCompleted = true;
			return moveCmd;
		}else{
			if(targetYaw == -100){
				gotoPoseCompleted = true;
				return moveCmd;
			}
			moveCmd.angular.z =    yawDiff;
			if(fabs(moveCmd.angular.z) > MAX_TURN_SPD){
				moveCmd.angular.z=MAX_TURN_SPD * sign;
			}
			if(fabs(moveCmd.angular.z) < MIN_TURN_SPD){
				moveCmd.angular.z = MIN_TURN_SPD * sign;
			}
			return moveCmd;
		}
	}
	
	// if goto pose is not completed try to go to the position
	if (!gotoPoseCompleted) {
	    targetYaw = atan2(diffy, diffx);
		sign = yawDiff < 0?-1:1;
		// if goto pose is not completed try to go to the position
		yawDiff = normalizeRad(targetYaw - robotYaw);
		sign = yawDiff < 0?-1:1;
		moveCmd.angular.z = yawDiff;

		if(fabs(moveCmd.angular.z) > MAX_TURN_SPD){
			moveCmd.angular.z=MAX_TURN_SPD * sign;
		}
		if(fabs(moveCmd.angular.z) < MIN_TURN_SPD){
			moveCmd.angular.z = MIN_TURN_SPD * sign;
		}

		if (fabs(yawDiff) < 0.2) {
			float linearK = 0.7;
			moveCmd.linear.x = linearK * sqrt(pow(diffy,2) + pow(diffx,2));
			if (moveCmd.linear.x > MAX_LINEAR_SPD){
				moveCmd.linear.x = MAX_LINEAR_SPD;
			}
		}
	}
	return moveCmd;
}



void isbeavioractive_func (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "go_near_beacon"){
        isbeavioractive = true;
    }else{
        isbeavioractive = false;
	behavior_changed = true;
    }

}


int main(int argc,char **argv)//0.3
{
	ros::init(argc,argv,"go_near_beacon");
   	ros::NodeHandle nh;
	ros::Rate rate(30);
	// read parameters
	ros::Subscriber BehaviorSubs = nh.subscribe( "/curbehavior", 1, isbeavioractive_func);
	//ros::Subscriber mapSubs = nh.subscribe("/map", 10, &MapHelper::processMap, &mapHelper);
	//ros::Subscriber imageSubs = nh.subscribe(prefix + "sensors/rgbImage", 10, &ImageHelper::processImage,        &imageHelper);
	//ros::Subscriber laserSubs = nh.subscribe( "/OmniPlatform001/sensors/scan", 10,  processLaserScan);
	ros::Subscriber poseSubs = nh.subscribe( "/amcl_pose", 10, updatePose); //geometry_msgs

    ros::Publisher velPub=nh.advertise<geometry_msgs::Twist>("/OmniPlatform001/cmd_vel",10);
    completedPub = nh.advertise<std_msgs::Bool>( "/Red_gotoPoseCompleted", 1);



    float targetx;
    float targety ;
    float targetyaw;
    
    gotoPoseCompleted = true ;
    robotposeGet = false ;

	geometry_msgs::Pose pose ;
	std::vector<geometry_msgs::Pose> init_poses ;
	pose.position.x = - 0.025 ;
	pose.position.y = 0.57 ;
	init_poses.push_back (pose) ;
	pose.position.x = - 1.45 ;
	pose.position.y = 0.57 ;
	init_poses.push_back (pose) ;
	pose.position.x = - 1.47;
	pose.position.y = 1.68 ;
	init_poses.push_back (pose) ;
	std::vector<geometry_msgs::Pose> poses ;
	//redx -0.121041redy 0.790346
	//greenx -1.40969greeny -0.319107

	//redx = -1.84851 + 0.25 ;
	//redy = 0.790346  + 0.25 ;
	//greenx = 2 ;//-1.40969 + 0.25;
	//greeny = 0;//-0.319107 + 0.25;
	poses = init_poses ; 
    bool go_back = false;
	behavior_changed = true;
    while(ros::ok()){	
		if(robotposeGet){
			if(!behavior_changed){//reset values
				poses = init_poses ; 
				go_back = true;
				gotoPoseCompleted = true;
			}

			if(isbeavioractive && behavior_changed){						
				if(go_back){
					geometry_msgs::Twist moveCmd;
					moveCmd.linear.x=-10;
					moveCmd.linear.y=0;
					moveCmd.angular.z=0;
					velPub.publish(moveCmd);
					usleep(900000);
					go_back = false;
				}else{ 	
					if(gotoPoseCompleted){
						targetx = robotX ;
						targety = robotY ;
						targetyaw = -100;
					   if(poses.size() == 0){
							std_msgs::Bool data;
							data.data = true;
							completedPub.publish(data);
							behavior_changed = false;
							gotoPoseCompleted = false ;
						}

						if (poses.size() == 1) {
							pose = poses.front();

							std::cout <<prefix << "sending command pose.x=" << pose.position.x << " pose.y=" << pose.position.y << std::endl;

							gotoPoseCompleted = false;

							// remove the first element
							poses.erase(poses.begin());
							targetx = pose.position.x ;
							targety = pose.position.y ;
							targetyaw = PI/2;

						}
						// if goto pose completed we can send new goto pose command
						if (poses.size() > 1) {
							pose = poses.front();

							std::cout << prefix <<  "sending command pose.x=" << pose.position.x << " pose.y=" << pose.position.y << std::endl;


							gotoPoseCompleted = false;

							// remove the first element
							poses.erase(poses.begin());
							targetx = pose.position.x ;
							targety = pose.position.y ;
							targetyaw = -100 ;
						}


					}
		    		velPub.publish(gotoPose(targetx,targety,targetyaw));
				}

			}

		}
		ros::spinOnce();
		rate.sleep();
	
	}

}
