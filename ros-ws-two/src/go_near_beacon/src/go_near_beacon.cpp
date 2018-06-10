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
bool pass;
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
            if(!isbeavioractive){pass = false;}
        isbeavioractive = true;

    }else{
        isbeavioractive = false;
	behavior_changed = true;
    }

}

void pass_func(const std_msgs::Bool::ConstPtr& BoolPointer ){
    if(BoolPointer->data ){
        pass = true;
        	//std::cout << prefix + "pass received" << std::endl; 
    }else{
        pass = false;
    }
}

int main(int argc,char **argv)//0.3
{
	ros::init(argc,argv,"go_near_beacon");
   	ros::NodeHandle nh;
	ros::Rate rate(30);
	// read parameters
ros::NodeHandle private_nh("~");
	if (!private_nh.getParam("prefix", prefix)){

	}

	ros::Subscriber BehaviorSubs = nh.subscribe( prefix + "curbehavior", 1, isbeavioractive_func);
	ros::Subscriber poseSubs = nh.subscribe( prefix + "amcl_pose", 10, updatePose); //geometry_msgs

    ros::Publisher velPub=nh.advertise<geometry_msgs::Twist>(prefix + "cmd_vel",10);
    completedPub = nh.advertise<std_msgs::Bool>( prefix + "Red_gotoPoseCompleted", 1);
    ros::Publisher passPub = nh.advertise<std_msgs::Bool>( "/Deliverpass", 5);
    ros::Subscriber passSubs = nh.subscribe( "/Beaconpass", 1,pass_func);
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
	poses = init_poses ; 
    bool go_back = false;
	behavior_changed = true;
	bool init = true;
	pass = false;
    while(ros::ok()){	
		if(robotposeGet){
			if(!behavior_changed){//reset values
				poses = init_poses ; 
				go_back = true;
				gotoPoseCompleted = true;
					pass = false;
			}

			if(isbeavioractive && behavior_changed){						
				if(go_back){
					geometry_msgs::Twist moveCmd;
					moveCmd.linear.x=-10;
					moveCmd.linear.y=0;
					moveCmd.angular.z=0;
					velPub.publish(moveCmd);
					usleep(1000000);
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

						if (!init){
							if (poses.size() == 3){
							//give signal
							if(!pass){ 						
						//std::cout << prefix + " waiting for pass" << std::endl;
						geometry_msgs::Twist moveCmd;
						moveCmd.linear.x=0;
						moveCmd.linear.y=0;
						moveCmd.angular.z=0;
						velPub.publish(moveCmd);
						ros::spinOnce();
							rate.sleep();  
						continue;}
						else{
						std::cout << prefix + "pass received" << std::endl; 
						
						pass= false;}

						
	
							}

						
						}
						
						if (poses.size() == 2){
						//give signal
						std_msgs::Bool data;
						data.data =true;
						passPub.publish(data);
						
						std::cout << prefix + " send pass" << std::endl; 
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
						init = false;

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
