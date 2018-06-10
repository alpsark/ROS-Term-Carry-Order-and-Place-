#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
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
bool behavior_changed ;
float robotX;
float robotY;
float robotYaw;
std::string prefix;
int order_num ;

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

void isbeavioractive_func1 (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "order"){
        isbeavioractive = true;
	prefix = "1" ;
    }else if(prefix == "1"){
        isbeavioractive = false;
	behavior_changed = true ;
    }

}

void isbeavioractive_func2 (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "order"){
        isbeavioractive = true;
	prefix = "2" ;
    }else if(prefix == "2"){
        isbeavioractive = false;
	behavior_changed = true ;
    }

}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"order");
   	ros::NodeHandle nh;
	ros::Rate rate(30);


	ros::Subscriber BehaviorSubs1 = nh.subscribe("/OmniPlatform001/curbehavior", 1, isbeavioractive_func1);
	ros::Subscriber BehaviorSubs2 = nh.subscribe("/OmniPlatform002/curbehavior", 1, isbeavioractive_func2);
	//ros::Subscriber imageSubs = nh.subscribe(prefix + "sensors/rgbImage", 10, &ImageHelper::processImage,        &imageHelper);
	//ros::Subscriber laserSubs = nh.subscribe(prefix + "sensors/scan", 10,  processLaserScan);
	ros::Subscriber poseSubs = nh.subscribe( "/OmniPlatform001/amcl_pose", 10, updatePose); //geometry_msgs
    ros::Publisher orderPub =nh.advertise<std_msgs::Float32>("/order",1);

    ros::Publisher took_orderPub1 =nh.advertise<std_msgs::Bool>("/OmniPlatform001/took_order",1);
    ros::Publisher took_orderPub2 =nh.advertise<std_msgs::Bool>("/OmniPlatform002/took_order",1);
    float targetx;
    float targety ;
    float targetyaw;
    float order = 1;
	//order += 1;

    behavior_changed = true ;
    while(ros::ok())
	{
        if(isbeavioractive  && behavior_changed){
		behavior_changed = false;
		std_msgs::Float32 ordernum;
		ordernum.data = order;
		orderPub.publish(ordernum);
		usleep(3000000);
		order = order + 1;
		std_msgs::Bool data;
		data.data = true ;
		if(prefix == "1"){took_orderPub1.publish(data);}
		else if(prefix == "2"){took_orderPub2.publish(data);}		
		std::cout << "took_order " <<  order -1 << std::endl; 
        }
		

		ros::spinOnce();
		rate.sleep();
	}




}
