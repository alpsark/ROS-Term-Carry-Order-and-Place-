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

#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.52359879
#define MIN_TURN_SPD 0.05

#define MAX_LINEAR_SPD 0.7
#define EPS 0.05

bool isbeavioractive ;

float robotX;
float robotY;
float robotYaw;
std::string prefix;

void updatePose(const geometry_msgs::PoseWithCovarianceStamped& NewPose){
	geometry_msgs::PoseWithCovariance  RobotPosewithcovariance = NewPose.pose;
	geometry_msgs::Pose  RobotPose = RobotPosewithcovariance.pose;
	robotX=RobotPose.position.x;
	robotY=RobotPose.position.y;
	robotYaw=tf::getYaw(RobotPose.orientation);	
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

	// check completed
	float diffx = targetPosex - robotX;
	float diffy = targetPosey - robotY;
	float yawDiff = normalizeRad(targetYaw - robotYaw);
	if (fabs(diffx) < EPS && fabs(diffy) < EPS && fabs(yawDiff) < EPS){
         return moveCmd;

	}	

	// if goto pose is not completed try to go to the position
	if(targetYaw = -100){
	targetYaw = atan2(diffy, diffx);
	}
	int sign = yawDiff < 0?-1:1;
	moveCmd.angular.z = yawDiff;

	if(fabs(moveCmd.angular.z) > MAX_TURN_SPD)
	{
		moveCmd.angular.z=MAX_TURN_SPD * sign;
	}
	if(fabs(moveCmd.angular.z) < MIN_TURN_SPD)
	{
		moveCmd.angular.z = MIN_TURN_SPD * sign;
	}

	if (fabs(yawDiff) < 0.2) {
		float linearK = 0.7;
		moveCmd.linear.x = linearK * sqrt(pow(diffy,2) + pow(diffx,2));
		if (moveCmd.linear.x > MAX_LINEAR_SPD)
		{
			moveCmd.linear.x = MAX_LINEAR_SPD;
		}
	}	

	return moveCmd;
}

void isbeavioractive_func (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "explore"){
        isbeavioractive = true;
    }else{
        isbeavioractive = false;
    }

}

int main(int argc,char **argv)
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

	//ros::Subscriber imageSubs = nh.subscribe(prefix + "sensors/rgbImage", 10, &ImageHelper::processImage,        &imageHelper);
	//ros::Subscriber laserSubs = nh.subscribe(prefix + "sensors/scan", 10,  processLaserScan);
	ros::Subscriber poseSubs = nh.subscribe(prefix +  "amcl_pose", 10, updatePose); //geometry_msgs

    ros::Publisher velPub=nh.advertise<geometry_msgs::Twist>(prefix + "cmd_vel",1);

    float targetx;
    float targety ;
    float targetyaw;
    while(ros::ok())
	{

        if(isbeavioractive){
            //apply wall following algorithm until beacon found
            targetx = robotX;
            targety = robotY ;
            targetyaw = 0 ;
	    velPub.publish(gotoPose(targetx,targety,targetyaw));
        }


		ros::spinOnce();
		rate.sleep();
	}




}
