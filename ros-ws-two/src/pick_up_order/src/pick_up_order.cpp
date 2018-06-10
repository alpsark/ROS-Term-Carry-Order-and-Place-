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
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <sensor_msgs/PointCloud2.h>
#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames_io.hpp"
#include "image_helper.h"

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/point_cloud.h>

#define UR10_DOF 6
#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.52359879
#define MIN_TURN_SPD 0.05

#define MAX_LINEAR_SPD 0.7
#define EPS 0.05

  ros::Subscriber cloudSubs;
  ros::Subscriber ballPoseSubs;

geometry_msgs::PoseStamped endEffectorPosition;

geometry_msgs::PoseArray cupPositions;

geometry_msgs::Pose calculatedPosition;
geometry_msgs::PoseArray cupTargetPositions;
std::string prefix;
ImageHelper imageHelper;
bool isCupTargetPositionsAvailable = false;
bool is_cup_taken =  false; // 
bool shelf_reached = false;
bool isCupPositionsAvailable ;

bool isbeavioractive ;
bool behavior_changed ;
float robotX;
float robotY;
float robotYaw;
float objectx;
float objecty;
float objectz;
float shelfx;
float shelfy;
float shelfz;
bool error_reset = true;
ros::Publisher beaconEnable ;

int iscube = true;

float calculateError (const geometry_msgs::PoseStamped& endEffectorPosition) {
    float error = 0;
    error += pow(calculatedPosition.position.x - endEffectorPosition.pose.position.x,2) * 10000;
    error += pow(calculatedPosition.position.y - endEffectorPosition.pose.position.y,2) * 10000;
    error += pow(calculatedPosition.position.z - endEffectorPosition.pose.position.z,2) * 10000;
    error = sqrt(error);

	if(error < 8 && error_reset) {
		if(shelf_reached){behavior_changed = false; 
			calculatedPosition.position.x  = 0;
			calculatedPosition.position.y   = 0;
			calculatedPosition.position.z  = 0;}
		if(is_cup_taken){shelf_reached = true; 		std::cout << "1" << std::endl ;}
		is_cup_taken = true; //
		error_reset = false;
		std::cout << "cup is taken" << std::endl ;
	}	else{
		    //std::cout << "error " << error << std::endl;
	}	
	
    return error;
}

void endEffectorPositionCallback(const geometry_msgs::PoseStampedConstPtr& endEffectorPositionMessage) {
    endEffectorPosition = *endEffectorPositionMessage;
    //std::cout << "Calculated Position   :" << "[ "<< calculatedPosition.position.x << " , " << calculatedPosition.position.y << " , " << calculatedPosition.position.z << " ]" << std::endl;
   // std::cout << "End Effector Position :" << "[ "<< endEffectorPositionMessage->pose.position.x << " , " << endEffectorPositionMessage->pose.position.y << " , " << endEffectorPositionMessage->pose.position.z << " ]" << std::endl;
    calculateError(*endEffectorPositionMessage);
}
KDL::Chain initChainUR10() {
  
 /*Base Position :[ -1.49143 , -1.59999 , 0.0446423 ] 
		joint positions are relative to base
		Joint1 Position :[ -0.0926735 , 1.07288e-06 , 0.083327 ]		
		Joint2 Position :[ -0.112951 , 0.000450611 , 0.695387 ]		
		Joint3 Position :[ -0.107075 , -0.000293493 , 1.26759 ]		
		Joint4 Position :[ -0.165607 , -0.000283599 , 1.32478 ]		
		Joint5 Position :[ -0.223011 , -0.000267982 , 1.38309 ]		
		End Effector Position :[ -0.333581 , -0.000296116 , 1.38248 ]		
 */
  KDL::Chain chain;
    	KDL::Segment s0 = KDL::Segment(KDL::Joint(KDL::Joint::RotZ), //z
                KDL::Frame(KDL::Rotation::RPY(0.0, 0.0 ,PI),
                          KDL::Vector(-0.0926735 , -1.07288e-06 , 0.083327) ) //ok
                    );
	
	KDL::Segment s1 = KDL::Segment(KDL::Joint(KDL::Joint::RotX), //-x
                KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(-0.0926735 - (-0.112951) ,1.07288e-06 - (0.000450611) , 0.695387 - 0.083327) )
                    );
                    
	KDL::Segment s2 = KDL::Segment(KDL::Joint(KDL::Joint::RotX), //-x
                KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(-0.112951  - (-0.107075),0.000450611 - (-0.000293493),1.26759-0.695387 ) )
                    );
                    

	KDL::Segment s3 = KDL::Segment(KDL::Joint(KDL::Joint::RotX), //-x
                KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(-0.107075 - (-0.165607),-0.000293493  -(-0.000283599 ),1.32478-1.26759) )
                    );

	KDL::Segment s4 = KDL::Segment(KDL::Joint(KDL::Joint::RotZ), //z
                KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(-0.165607 - (-0.223011), -0.000283599 - (-0.000267982),1.38309-1.32478) )
                    );

	KDL::Segment s5 = KDL::Segment(KDL::Joint(KDL::Joint::RotX), //-x
                KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0),
                          KDL::Vector(-0.223011 - (-0.333581) ,-0.000267982 - (-0.000296116) ,1.38248-1.38309) )
                    );
	
    chain.addSegment(s0);
    chain.addSegment(s1);
    chain.addSegment(s2);
    chain.addSegment(s3);
    chain.addSegment(s4);
    chain.addSegment(s5);


    return chain;
}

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


void isbeavioractive_func1 (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "pick_up_order"){
        isbeavioractive = true;
	std_msgs::Bool data;
	data.data = true;
	beaconEnable.publish(data);
	prefix = "1" ;
    }else if(prefix == "1"){
        isbeavioractive = false;
   		behavior_changed = true;
    }
  //std::cout <<"omni1" << prefix  << std::endl;
}

void isbeavioractive_func2 (const std_msgs::String::ConstPtr& StringPointer ){
    if(StringPointer->data == "pick_up_order"){
        isbeavioractive = true;
	std_msgs::Bool data;
	data.data = true;
	beaconEnable.publish(data);
	prefix = "2" ;
    }else if(prefix == "2"){
        isbeavioractive = false;
   		behavior_changed = true;
    }
  //std::cout << "omni1" << prefix  << std::endl;
}


class StateEstimation {
  ros::Subscriber cloudSubs;
  ros::Subscriber ballPoseSubs;
  ros::NodeHandle nh;

  tf::TransformListener tfListener;

  bool isAdded;
  
  //pcl::visualization::PCLVisualizer* viewer;

 public:
  StateEstimation() {
    cloudSubs = nh.subscribe("/tools/sensors/greenBeaconDepth", 1,
                             &StateEstimation::pointCloudCallback, this);


    isAdded = false;

    //viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
   // viewer->initCameraParameters();
     // viewer->setBackgroundColor (0, 0, 0);
     // viewer->addCoordinateSystem (1.0);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

	if(!isCupPositionsAvailable){
		 ros::Time t = ros::Time(0);
		// wait from transform
		tfListener.waitForTransform("greenBeacon_base", "/tools/sensors/greenBeaconDepth", t,
		                            ros::Duration(1.0));

		// convert point cloud frame to beacon frame

		sensor_msgs::PointCloud2 transformedPoints;
		pcl_ros::transformPointCloud("greenBeacon_base", *msg, transformedPoints,
		                             tfListener);
		//pcl::PointCloud<pcl::PointXYZ> *pointCloud = new pcl::PointCloud<pcl::PointXYZ>();
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(transformedPoints, *pointCloud);
	
		std::cout << "size of pcl= " << pointCloud->points.size () << std::endl;
		// DO: detect ball here

		//plane detection

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (pointCloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		}

		/*std::cout << "Model coefficients: " << coefficients->values[0] << " " 
				                            << coefficients->values[1] << " "
				                          	<< coefficients->values[2] << " " 
				                         	<< coefficients->values[3] << std::endl;

	  	std::cout << "Model inliers: " << inliers->indices.size () << std::endl; */

		//for (size_t i = 0; i < inliers->indices.size (); ++i)
		//    inliers->indices[i] << "    " << pointCloud->points[inliers->indices[i]].x << " "
		//                                  << pointCloud->points[inliers->indices[i]].y << " "
		//                                  << pointCloud->points[inliers->indices[i]].z << std::endl;
	   
	 	//remove indices of plane
	 	pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(pointCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*pointCloud);

		std::cout << "size after plane removal = " << pointCloud->points.size () << std::endl;
	
		//plane removed view	
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr constPointCloud(pointCloud);
	 
		//viewer->removePointCloud("cloud");
		//viewer->addPointCloud(constPointCloud, "cloud");
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		 
		//viewer->spinOnce(100);

		//limit x axis to detect only ball

		int cloudsize= 0 ; 
		for (int i = 0; i < pointCloud->points.size (); i++) {
			//cout << "x:" << pointCloud->points[i].x  << "y:" << pointCloud->points[i].y << std::endl;
			if(  pointCloud->points[i].y-0.949998 < -1.09 and pointCloud->points[i].y-0.949998 > -1.2 and pointCloud->points[i].x-1.1 <-1.11){

			}
			else if(  pointCloud->points[i].x-1.1 < -0.84  and pointCloud->points[i].x-1.1 >-1.0){
				if( pointCloud->points[i].y-0.949998 < -0.91 and pointCloud->points[i].y-0.949998 > -1.3){
				cout << "x:" << pointCloud->points[i].x  << "y:" << pointCloud->points[i].y << std::endl;
				  cloudsize++ ; 

				 }
				}
	   	}

		//create new cloud data for ball
	  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// Fill in the cloud data
	  	cloud->width  = cloudsize;
	  	cloud->height = 1;
	  	cloud->points.resize (cloud->width * cloud->height);


		cloudsize= 0 ; 
		float measured_x = 0;
		float measured_y = 0;
		float measured_z = 0;
		for (int i = 0; i < pointCloud->points.size (); i++) {
if(  pointCloud->points[i].y-0.949998 < -0.952 and pointCloud->points[i].y-0.949998 > -1.2 and pointCloud->points[i].x-1.1 <-1.11){

			}
			else if( pointCloud->points[i].x-1.1 < -0.84  and pointCloud->points[i].x-1.1 >-1.0){
				if( pointCloud->points[i].y-0.949998 < -0.91 and pointCloud->points[i].y-0.949998 > -1.3){
				cloud->points[cloudsize].x = pointCloud->points[i].x ;
				  measured_x = measured_x + pointCloud->points[i].x;  
				  cloud->points[cloudsize].y = pointCloud->points[i].y ;
				  measured_y = measured_y + pointCloud->points[i].y ;
	   			  cloud->points[cloudsize].z = pointCloud->points[i].z ;
				  measured_z = measured_z + 	 pointCloud->points[i].z;
				  cloudsize++ ; 

				 }
				}
		}
	
		if(cloudsize > 0  ){//greenbeacon Position :[ -1.1 , -0.949998 , 0.25 ]yaw 0

			objectx = measured_x/cloudsize ;
			objecty = measured_y/cloudsize ;
			objectz = measured_z/cloudsize ;
		
			if(iscube){//cylynder
				iscube = false;
				shelfz = +8.5064e-01;	
			}else{//cube
				iscube = true;
				shelfz = +1.2306e+00;

			}//local x:0.244029 y:-0.164809 z:0.1055
			//global x:0.494029 y:-1.11481 z:-0.99
			//real -0.85189e -1.1299e+00 +0.33054
			std::cout << "local" << " x:" << objectx << " y:" << objecty << " z:" << objectz ; // 
			objectz = 0.25 + objectz ;  //
			//float temp = objecty;
			objecty = -0.949998 + objecty ;
			objectx = -1.1 + objectx ;
			isCupPositionsAvailable = true;
			std::cout << "global" << " x:" << objectx << " y:" << objecty << " z:" << objectz ; // 
		
		//	viewer->removePointCloud("cloud");
		//	viewer->addPointCloud(cloud, "cloud");
		//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			  
		//	viewer->spinOnce(100);
		
			//Base Position :[ -1.49143 , -1.59999 , 0.0446423 ]yaw -3.14157

			calculatedPosition.position.x =  -1.4914e+00 -  objectx;
			calculatedPosition.position.y = -1.59999  -  objecty;
			calculatedPosition.position.z = objectz - 0.0446423 + 0.1;
		}
	}
	}
 };


int main(int argc,char **argv)
{
	
  std::cout << "PLC Start..." << std::endl;
	ros::init(argc,argv,"pick_up_order");



   	ros::NodeHandle nh;
	ros::Rate rate(30);
	

	ros::Publisher joints[6];
    joints[0] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint001",1000);
    joints[1] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint002",1000);
    joints[2] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint003",1000);
    joints[3] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint004",1000);
    joints[4] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint005",1000);
    joints[5] = nh.advertise<std_msgs::Float32>("/UR10/jointAngles/joint006",1000);


	 for(int index=0;index<UR10_DOF;index++){
		    std_msgs::Float32 jointMessage;
		    jointMessage.data = 0;
		    joints[index].publish(jointMessage);
	}

  ros::Subscriber endEffectorPositionSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/UR10/positions/endEffectorPosition",1000,endEffectorPositionCallback);

   // ros::Subscriber cupPositionsSubscriber = nh.subscribe<geometry_msgs::PoseArray>("/task/positions/cups",1000,cupPositionaCallback);
    //ros::Subscriber cupTargetPositionsSubscriber = nh.subscribe<geometry_msgs::PoseArray>("/task/positions/cuptargets",1000,cupTargetPositionaCallback);
    ros::Publisher gripper = nh.advertise<std_msgs::Float32>("/UR10/gripper/closingAngle",1000);
    ros::Publisher PickedupPub1 =nh.advertise<std_msgs::Bool>("/OmniPlatform001/Pickedupobject",1);
    ros::Publisher PickedupPub2 =nh.advertise<std_msgs::Bool>("/OmniPlatform002/Pickedupobject",1);
	//ros::Rate loop(10);

    KDL::Chain chain = initChainUR10();
    geometry_msgs::PoseArray goalPositions;
	ros::Subscriber BehaviorSubs1 = nh.subscribe("/OmniPlatform001/curbehavior", 1, isbeavioractive_func1);
	ros::Subscriber BehaviorSubs2 = nh.subscribe("/OmniPlatform002/curbehavior", 1, isbeavioractive_func2);
	ros::Subscriber imageSubs = nh.subscribe("/tools/sensors/greenBeaconRGB", 10, &ImageHelper::processImage,        &imageHelper);
	ros::Subscriber poseSubs = nh.subscribe( "/OmniPlatform001/amcl_pose", 10, updatePose); //geometry_msgs
	beaconEnable = nh.advertise<std_msgs::Bool>("/tools/sensors/greenBeaconEnable", 1);
    
    StateEstimation stateEstimation;
	int gripper_count = 0 ;
	bool cup_gripped = false;
	std_msgs::Bool data;
	data.data = true;
	behavior_changed = true;
	isCupPositionsAvailable = false;
	shelfx = -1.95;
    while(ros::ok()){

		if(!behavior_changed){//reset values
			shelf_reached = false;
			gripper_count = 0;
			isCupPositionsAvailable = false;
			isCupTargetPositionsAvailable = false ;
			is_cup_taken =false;
			cup_gripped = false;
			error_reset = true;
		}
		if(shelf_reached){
			if(prefix == "1"){PickedupPub1.publish(data);}
			else if(prefix == "2"){PickedupPub2.publish(data);}
			behavior_changed =false ;
			std_msgs::Float32 jointMessage;
			KDL::ChainFkSolverPos_recursive forwardKinematicSolver = KDL::ChainFkSolverPos_recursive(chain);
			KDL::Frame goalEndEffectorPosition;
			jointMessage.data = 0;
			gripper.publish(jointMessage);
			calculatedPosition.position.x =  -0.333581 ;
			calculatedPosition.position.y = -0.000296116;
			calculatedPosition.position.z = 1.38248 ;
			goalEndEffectorPosition = KDL::Frame(
						        KDL::Rotation::RPY(PI/2,0,PI),
						        KDL::Vector(calculatedPosition.position.x,calculatedPosition.position.y,calculatedPosition.position.z));
			// You can use this matrix for ChainIkSolverPos_LMA
			Eigen::Matrix<double,6,1> L;
			L(0)=1;L(1)=1;L(2)=1;
			L(3)=0.01;L(4)=0.01;L(5)=0.01;

			//create iksolver
			KDL::ChainIkSolverPos_LMA iksolver = KDL::ChainIkSolverPos_LMA(chain,L );

			// Create joint arrays
			unsigned int nj = chain.getNrOfJoints();
			KDL::JntArray initialjointpositions = KDL::JntArray(nj);
			KDL::JntArray jointpositions = KDL::JntArray(nj);
			int ret = iksolver.CartToJnt(initialjointpositions, goalEndEffectorPosition, jointpositions);
			// Create the frame that will contain the results
			KDL::Frame cartpos;   
			// Calculate forward position kinematics
			bool kinematics_status;
			kinematics_status = forwardKinematicSolver.JntToCart(jointpositions,cartpos);
			if(kinematics_status>=0){
			
			}else{
				printf("%s \n","Error: could not calculate forward kinematics :(");
			}

			/*
			* DO: Publish the found joint angles.
			*/
			if(ret == 0) {

				jointMessage.data = jointpositions(0);
				joints[0].publish(jointMessage);
				jointMessage.data = jointpositions(1);
				joints[1].publish(jointMessage);
				jointMessage.data = jointpositions(2);
				joints[2].publish(jointMessage);
				jointMessage.data = jointpositions(3);
				joints[3].publish(jointMessage);
				jointMessage.data = jointpositions(4);
				joints[4].publish(jointMessage);
				jointMessage.data = jointpositions(5);
				joints[5].publish(jointMessage);
			}else{ 	
			 std::cout << ret ;   printf("%s \n","Error: could not calculate inverse kinematics :("); 
			}

		    error_reset = true ;
		ros::spinOnce();
		rate.sleep();	
		    continue;					
		}
		
        if(isbeavioractive && behavior_changed){ 
        		//findcuptarget pos
        	if(isCupPositionsAvailable){
		    	 if(imageHelper.isbluefound){
	    	 			//std::cout << "blue " << std::endl ;
			    		shelfy = -1.9046e+00;
		    	 }else if(imageHelper.isgreenfound){
						//std::cout << "green " << std::endl ;
						shelfy = -1.2546e+00;
		    	 }
		    	 isCupTargetPositionsAvailable = true;
        	}
    		
			if(isCupPositionsAvailable && isCupTargetPositionsAvailable) {   
				KDL::ChainFkSolverPos_recursive forwardKinematicSolver = KDL::ChainFkSolverPos_recursive(chain);
				KDL::Frame goalEndEffectorPosition;
				std_msgs::Float32 jointMessage;
				jointMessage.data = 5.5;

				if(is_cup_taken) {
					gripper.publish(jointMessage); gripper_count++ ; 
					if(gripper_count > 15) {
						cup_gripped = true;
							//publish cup taken

						
					}
				}
			
				if (cup_gripped){		//Base Position :[ -1.49143 , -1.59999 , 0.0446423 ]yaw -3.14157

					calculatedPosition.position.x =  -1.4914e+00 - shelfx ;
					calculatedPosition.position.y = -1.59999 - shelfy;
					calculatedPosition.position.z = shelfz - +0.0446423;			
				}


				if(cup_gripped){ 
					goalEndEffectorPosition = KDL::Frame(
					        KDL::Rotation::RPY(0,0,0),
					        KDL::Vector(calculatedPosition.position.x,calculatedPosition.position.y,calculatedPosition.position.z));
					    error_reset = true ;
					  //-0.714789 , -0.155105 , 0.88193)
				}else{ goalEndEffectorPosition = KDL::Frame(
						        KDL::Rotation::RPY(0,PI/2,0),
						        KDL::Vector( calculatedPosition.position.x,calculatedPosition.position.y,calculatedPosition.position.z));			        
						        							        							        
				}
				   

				// You can use this matrix for ChainIkSolverPos_LMA
				Eigen::Matrix<double,6,1> L;
				L(0)=1;L(1)=1;L(2)=1;
				L(3)=0.01;L(4)=0.01;L(5)=0.01;

				//create iksolver
				KDL::ChainIkSolverPos_LMA iksolver = KDL::ChainIkSolverPos_LMA(chain,L );

				// Create joint arrays
				unsigned int nj = chain.getNrOfJoints();
				KDL::JntArray initialjointpositions = KDL::JntArray(nj);
				KDL::JntArray jointpositions = KDL::JntArray(nj);


				int ret = iksolver.CartToJnt(initialjointpositions, goalEndEffectorPosition, jointpositions);

				 // Create the frame that will contain the results
				KDL::Frame cartpos;   
			 
				// Calculate forward position kinematics
				bool kinematics_status;
				kinematics_status = forwardKinematicSolver.JntToCart(jointpositions,cartpos);
				if(kinematics_status>=0){
					//cartpos.M rot matrix
					//cartpos.p end effector pos
					//std::cout << "KDL end effector :"<< cartpos.p <<std::endl;
				    //printf("%s \n","Succes");
				}else{
					printf("%s \n","Error: could not calculate forward kinematics :(");
				}

				 /*
					 * DO: Publish the found joint angles.
					 */
				if(ret == 0) {

					jointMessage.data = jointpositions(0);
					joints[0].publish(jointMessage);
					jointMessage.data = jointpositions(1);
					joints[1].publish(jointMessage);
					jointMessage.data = jointpositions(2);
					joints[2].publish(jointMessage);
					jointMessage.data = jointpositions(3);
					joints[3].publish(jointMessage);
					jointMessage.data = jointpositions(4);
					joints[4].publish(jointMessage);
					jointMessage.data = jointpositions(5);
					joints[5].publish(jointMessage);}
				else{ 	 std::cout << ret ;   printf("%s \n","Error: could not calculate inverse kinematics :("); 			calculatedPosition.position.z = calculatedPosition.position.z + 0.01; }
			
			}
		}
	
		ros::spinOnce();
		rate.sleep();
	}

}
