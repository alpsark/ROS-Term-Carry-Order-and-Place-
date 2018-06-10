#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "map_helper.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

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
MapHelper mapHelper;
float redx;
float redy;
float greenx;
float greeny;
bool isredfound;
bool isgreenfound;
ImageHelper imageHelper;
std::string redfoundby ;
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
		std_msgs::Bool data;
		if(fabs(yawDiff) < 2 * EPS){
			gotoPoseCompleted = true;
			data.data = gotoPoseCompleted;
			completedPub.publish(data);
			return moveCmd;
		}else{
			if(targetYaw == -100){
				gotoPoseCompleted = true;
				data.data = gotoPoseCompleted;
				completedPub.publish(data);
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

void processLaserScan(const sensor_msgs::LaserScanConstPtr& laserScanConstPtr)
{
	float obstacleThreshold = 0.3;
	float avgDist = 0;
	int rangeCount = 0;
	float rangeThreshold = 0.02;
	isrobot_scan_all_area = true;;
    obstacle_detected = false;
	for(int i=0; i<laserScanConstPtr->ranges.size()-1; i++){
		    if(laserScanConstPtr->ranges[i] < laserScanConstPtr->range_max && laserScanConstPtr->ranges[i] > laserScanConstPtr->range_min ) // is current scan valid?
		    {
	            if(i>laserScanConstPtr->ranges.size()/2 - 30 || i<laserScanConstPtr->ranges.size()/2 + 30){

	                if (laserScanConstPtr->ranges[i] < obstacleThreshold) {
                        obstacle_detected = true;
		                //std::cout<<"there is obstacle" << std::endl;
	                }
			            //avgDist += 
			            //rangeCount++;
		        }
                if(fabs(laserScanConstPtr->ranges[i]-laserScanConstPtr->ranges[i+1]) > rangeThreshold){
                    //does laserscan data comes from a closed loop
			            isrobot_scan_all_area = false ;
                }
                
            }else
	    	{
			isrobot_scan_all_area = false ;
    		}
	}
	//avgDist = avgDist / rangeCount;
}

void isbeavioractive_func (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "explore"){
        isbeavioractive = true;
    }else{
        isbeavioractive = false;
    }

}

bool isleftoccupied (){	
	return mapHelper.isOccupied(robotX- 0.5*sin(robotYaw) - 0.25*cos(robotYaw), robotY+ 0.5*cos(robotYaw) - 0.25*sin(robotYaw)) ;
		//std::cout << prefix << "left occupied" << std::endl ; 
	
}

bool isrightoccupied (){	
	return mapHelper.isOccupied(robotX+ 0.5*sin(robotYaw) - 0.25*cos(robotYaw) , robotY- 0.5*cos(robotYaw) - 0.25*sin(robotYaw)) ;
		//std::cout << prefix << "right occupied" << std::endl ; 
	
}

bool isaheadoccupied (){	
	return mapHelper.isOccupied(robotX+ 0.5*cos(robotYaw) , robotY+ 0.5*sin(robotYaw) ) ;
		//std::cout << prefix << "right occupied" << std::endl ; 
}

void redfound (const geometry_msgs::Pose& _targetPose ){
    redx =_targetPose.position.x;
    redy = _targetPose.position.y; 
     if(!isredfound){
    	isredfound = true;
        redfoundby = "Not You";
    }  

}
void greenfound (const geometry_msgs::Pose& _targetPose ){
    isgreenfound = true;
    greenx =_targetPose.position.x;
    greeny = _targetPose.position.y;
}

void findBeaconLocation(std::string color){
    float iteration = 0.1 ;
    float i = 0;
    while(true){
       if( mapHelper.isReallyOccupied(robotX+ i*cos(robotYaw) , robotY+ i*sin(robotYaw) )) {
          if(color == "green"){
            greenx = robotX+ (i)*cos(robotYaw);
            greeny = robotY+ (i)*sin(robotYaw);
          }else if(color == "red"){
            redx = robotX + (i)*cos(robotYaw);
            redy = robotY+ (i)*sin(robotYaw);
            
          }
          return;
        }
        i += iteration ; 
    }

}

int main(int argc,char **argv)//0.3
{
	ros::init(argc,argv,"explore");
   	ros::NodeHandle nh;
	ros::Rate rate(30);
	// read parameters
	ros::NodeHandle private_nh("~");
	if (!private_nh.getParam("prefix", prefix)){
		std::cout << "Error no prefix=" << std::endl;
		return -1; 
	}

	ros::Subscriber BehaviorSubs = nh.subscribe(prefix + "curbehavior", 1, isbeavioractive_func);
	ros::Subscriber mapSubs = nh.subscribe("/map", 10, &MapHelper::processMap, &mapHelper);
	ros::Subscriber imageSubs = nh.subscribe(prefix + "sensors/rgbImage", 10, &ImageHelper::processImage,        &imageHelper);
	ros::Subscriber laserSubs = nh.subscribe(prefix + "sensors/scan", 10,  processLaserScan);
	ros::Subscriber poseSubs = nh.subscribe(prefix +  "amcl_pose", 10, updatePose); //geometry_msgs

    ros::Subscriber redfoundSubs =nh.subscribe("/redfoundPub",1,redfound);
    ros::Subscriber greenfoundSubs =nh.subscribe("/greenfoundPub",1,greenfound);

    ros::Publisher redfoundPub =nh.advertise<geometry_msgs::Pose>("/redfoundPub",1);
    ros::Publisher greenfoundPub =nh.advertise<geometry_msgs::Pose>("/greenfoundPub",1);
    ros::Publisher velPub=nh.advertise<geometry_msgs::Twist>(prefix + "cmd_vel",10);
    completedPub = nh.advertise<std_msgs::Bool>(prefix + "Explore_gotoPoseCompleted", 1);
    float targetx;
    float targety ;
    float targetyaw;
    bool go_straight = false;
    gotoPoseCompleted = true ;
    robotposeGet = false ;
	isredfound = false;
	isgreenfound = false;
    while(ros::ok()){	
		if(robotposeGet){
		    if(isbeavioractive){

			if(imageHelper.isgreenfound ){
					//geometry_msgs::Pose pose;            
					//findBeaconLocation(imageHelper.greenx*(PI/8),"green");
					isgreenfound = true;  
					//pose.position.x=greenx;
					//pose.position.y=greeny;
					//std::cout << "greenx " << greenx << "greeny "<< greeny << std::endl; 
					//greenfoundPub.publish(pose);
					targetx = robotX ;
					targety = robotY;
					targetyaw = robotYaw + fabs(imageHelper.greenx) * 2;
					if(fabs(imageHelper.greenx)*2 < 2 * EPS){
						targetyaw = robotYaw ;
						geometry_msgs::Pose pose;
					    findBeaconLocation("green");
						pose.position.x=greenx;
						pose.position.y=greeny;
					    std::cout << "greenx " << greenx << "greeny "<< greeny << std::endl; 
					    greenfoundPub.publish(pose);


					}
				}
				if(imageHelper.isredfound ){
					//geometry_msgs::Pose pose;
					//findBeaconLocation(imageHelper.redx*(PI/4),"red");
					//pose.position.x=redx;
					//pose.position.y=redy;
					//std::cout << "redx " << redx << "redy "<< redy << std::endl; 
					//redfoundby = prefix;
					isredfound = true;  
					//redfoundPub.publish(pose);

					targetx = robotX ;
					targety = robotY;
					targetyaw = robotYaw - fabs(imageHelper.redx) *2;
					std::cout << "red " << fabs(imageHelper.redx)*2 << std::endl;
					if(fabs(imageHelper.redx)*2  < 2 * EPS){
						targetyaw = robotYaw ;
						geometry_msgs::Pose pose;
					    findBeaconLocation("red");
						pose.position.x=redx;
						pose.position.y=redy;
					    std::cout << "redx " << redx << "redy "<< redy << std::endl; 
					    redfoundPub.publish(pose);
					}
				}

			    if(gotoPoseCompleted  && (!isredfound || redfoundby!=prefix) && !isgreenfound ){
		            go_straight = false;
		            gotoPoseCompleted = false;
		            targetx = robotX;
		            targety = robotY ;
		            targetyaw = robotYaw ;
		            if(prefix.find("2") != std::string::npos){//hug left wall
				        if(isleftoccupied ()){
			                 if(isaheadoccupied ()){//turn left
			                   targetyaw = robotYaw + PI/2 ; 
			                   std::cout << prefix << "left" << std::endl;
		                      } else if(isrobot_scan_all_area){
			                   targetyaw = robotYaw + PI/2 ; 
			                   std::cout << prefix << "scan_all_area" << std::endl;                       
		                      }else{//go straight
		                       go_straight = true;
			                   targetx = robotX + 1*cos(robotYaw) ; 
		                       targety = robotY+ 1*sin(robotYaw) ;
			                   std::cout << prefix << "Straight" << std::endl;
			                 }
		                } else if(isrobot_scan_all_area){
			                   targetyaw = robotYaw - PI/2 ; //turn right
			                   std::cout << prefix << "scan_all_area" << std::endl;                         
		                }else{//turnleft , 
			                   targetx = robotX- 1*sin(robotYaw)  ; 
		                       targety = robotY+ 1*cos(robotYaw) ;
			                   targetyaw = robotYaw + PI/2 ; //turn left
			                   std::cout << prefix << "Goleft" << std::endl;                      
		                }
		            }else{
		                if(isrightoccupied ()){//hug right wall
			                 if(isaheadoccupied ()){
			                   targetyaw = robotYaw - PI/2 ; //turn right
			                   std::cout << prefix << "right" << std::endl;
		                    }else if(isrobot_scan_all_area){ //turn left
			                   targetyaw = robotYaw - PI/2 ; //turn right
			                   std::cout << prefix << "scan_all_area" << std::endl; 
		                    }else{//go straight
		                       go_straight = true;
			                   targetx = robotX + 1*cos(robotYaw) ; 
		                       targety = robotY+ 1*sin(robotYaw) ;
			                   std::cout << prefix << "Straight" << std::endl;
			                 }
		                }
		                else if(isrobot_scan_all_area){ //turn left
			                   targetyaw = robotYaw + PI/2 ;
			                   std::cout << prefix << "scan_all_area" << std::endl;   
		                }else{//turnright 
			                   targetx = robotX+ 1*sin(robotYaw)  ; 
		                       targety = robotY- 1*cos(robotYaw) ;
			                   targetyaw = robotYaw - PI/2 ; 
			                   std::cout << prefix << "Goright" << std::endl;                      
		                }   
		            }
                }else{
                    if(obstacle_detected && fabs(robotYaw - targetyaw) < 2*EPS){
                            targetx = robotX;
                            targety = robotY ;
                            targetyaw = robotYaw ;
		                    std::cout<< prefix << "there is obstacle" << std::endl;
                    }
                }
                //apply wall following algorithm until beacon found
                velPub.publish(gotoPose(targetx,targety,targetyaw));
                
            }

   	    }
		ros::spinOnce();
		rate.sleep();
	
	}

}
