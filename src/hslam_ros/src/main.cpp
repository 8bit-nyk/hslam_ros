/**
* This file is part of FLSAM_ROS.
Based on and inspired by DSO project by Jakob Engel

*/

#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>



#include "IOWrapper/Output3DWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace HSLAM;

std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string saveFile = "";
std::string vocabPath = "";
bool useSampleOutput=false;
int mode = 0;
int preset= 0;

ros::Publisher map_pub ;
ros::Publisher pose_pub ;
ros::Publisher path_pub ;
ros::Publisher odom_pub;
ros::Subscriber odom_filtered_sub;

ros::Publisher path_filtered_pub;
nav_msgs::Path path_filtered;
SE3 filtered_pose;
void parseArgument(char* arg)
{
	int option;
	char buf[1000];
	if(1==sscanf(arg,"savefile=%s",buf))
	{
		saveFile = buf;
		printf("saving to %s on finish!\n", saveFile.c_str());
		return;
	}

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	if(1==sscanf(arg,"LoopClosure=%d",&option))
	{
		if(option==1)
		{
			LoopClosure = true;
			printf("hslam_ros :LOOP CLOSURE IS TURNED ON!\n");
		}
		return;
	}

	if(1==sscanf(arg,"vocabPath=%s",buf))
	{
		vocabPath = buf;
		printf("hslam_ros : loading Vocabulary from %s!\n", vocabPath.c_str());
		return;
	}

	if (1==sscanf(arg,"mode=%d",&option))
	{
		if(option==1)
		{
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			
		}
		if(option==2)
		{
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_minGradHistAdd = 3;

		}
	
	}
	if (1==sscanf(arg,"preset=%d",&option))
	{
		if(option == 0 || option == 1)
		{
			printf("DEFAULT settings:\n"
					"- %s real-time enforcing\n"
					"- 2000 active points\n"
					"- 5-7 active frames\n"
					"- 1-6 LM iteration each KF\n"
					"- original image resolution\n", preset==0 ? "no " : "1x");
		}
		else if(option == 2 || option == 3)
		{
			printf("FAST settings:\n"
					"- %s real-time enforcing\n"
					"- 800 active points\n"
					"- 4-6 active frames\n"
					"- 1-4 LM iteration each KF\n"
					"- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");
			setting_desiredImmatureDensity = 600;
			setting_desiredPointDensity = 800;
			setting_minFrames = 4;
			setting_maxFrames = 6;
			setting_maxOptIterations=4;
			setting_minOptIterations=1;

			benchmarkSetting_width = 424;
			benchmarkSetting_height = 320;

			setting_logStuff = false;
		}
	}
	if(LoopClosure && !vocabPath.empty())
	{
		Vocab.load(vocabPath.c_str());
		printf("Loop Closure ON and loading Vocabulary from %s!\n", vocabPath.c_str());
		if (Vocab.empty())
		{
			printf("failed to load vocabulary! Exit\n");
			exit(1);
		}
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}


FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;



void publishResults() {        
		nav_msgs::Path path;
		nav_msgs::Odometry odom_msg;
		geometry_msgs::PoseWithCovarianceStamped pose_stamped;
		geometry_msgs::PoseStamped pose_stamped_msg;
		sensor_msgs::PointCloud2 map;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		std::vector<SE3> points;
		std::vector<Eigen::Vector3f> map_points;

		points=fullSystem->getPath();
		SE3 velocity = fullSystem->getVelocity();
		
		path.header.frame_id="odom";
		pose_stamped.header.frame_id="odom";
		pose_stamped.header.stamp = ros::Time::now();
		// Populate the Odometry message
		odom_msg.header.stamp = ros::Time::now();
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";
		
		for (size_t i = 0; i < points.size(); i++)
		{
			pose_stamped.pose.pose.position.x=points[i].translation().x();
			pose_stamped.pose.pose.position.y=points[i].translation().y();
			pose_stamped.pose.pose.position.z=points[i].translation().z();
			pose_stamped.pose.pose.orientation.x=points[i].so3().unit_quaternion().x();
			pose_stamped.pose.pose.orientation.y=points[i].so3().unit_quaternion().y();
			pose_stamped.pose.pose.orientation.z=points[i].so3().unit_quaternion().z();
			pose_stamped.pose.pose.orientation.w=points[i].so3().unit_quaternion().w();
			pose_stamped.pose.covariance = {0.1, 0, 0, 0, 0, 0,
											0, 0.1, 0, 0, 0, 0,
											0, 0, 0.1, 0, 0, 0,
											0, 0, 0, 0.5, 0, 0,
											0, 0, 0, 0, 0.5, 0,
											0, 0, 0, 0, 0, 0.5};// Populate the Path message
			// path.poses.push_back(pose_stamped.pose);
			// Convert PoseWithCovarianceStamped to PoseStamped for path
			pose_stamped_msg.header = pose_stamped.header;
			pose_stamped_msg.pose = pose_stamped.pose.pose;
			path.poses.push_back(pose_stamped_msg);

			// Populate the Odometry message
			// Directly assign pose (position, orientation and covariance) from PoseWithCovarianceStamped to Odometry message
    		odom_msg.pose = pose_stamped.pose;
			// Retrieve and set the twist (linear and angular velocity)
			odom_msg.twist.twist.linear.x = velocity.translation().x();
			odom_msg.twist.twist.linear.y = velocity.translation().y();
			odom_msg.twist.twist.linear.z = velocity.translation().z();
			odom_msg.twist.twist.angular.x = velocity.so3().log().x();
			odom_msg.twist.twist.angular.y = velocity.so3().log().y();
			odom_msg.twist.twist.angular.z = velocity.so3().log().z();
			// Set the covariance for twist
			odom_msg.twist.covariance = {0.01, 0, 0, 0, 0, 0,
											0, 0.01, 0, 0, 0, 0,
											0, 0, 0.01, 0, 0, 0,
											0, 0, 0, 0.01, 0, 0,
											0, 0, 0, 0, 0.01, 0,
											0, 0, 0, 0, 0, 0.01};
		

		}
	
		
	
		// Publish the Odometry message
		odom_pub.publish(odom_msg);
		path_pub.publish(path);
		pose_pub.publish(pose_stamped);
		//Write to file
		std::ofstream outfile;
		outfile.open("/home/nykvm/hslam_ros_ws/results/hslam_result.txt", std::ios_base::app);
		outfile << pose_stamped.header.stamp << " " << pose_stamped.pose.pose.position.x << " " << pose_stamped.pose.pose.position.y << " " << pose_stamped.pose.pose.position.z << " " << 
		pose_stamped.pose.pose.orientation.x << " " << pose_stamped.pose.pose.orientation.y << " " << pose_stamped.pose.pose.orientation.z << " " << pose_stamped.pose.pose.orientation.w << std::endl;
		outfile.close();
		// Publish the path
    	path_filtered_pub.publish(path_filtered);

		map_points=fullSystem->getMap();
		for (size_t i = 0; i < map_points.size(); i++)
		{
			pcl::PointXYZ point;
			point.x=map_points[i].x();
			point.y=map_points[i].y();
			point.z=map_points[i].z();
			cloud.push_back(point);
		}
		pcl::toROSMsg(cloud, map);
		map.header.frame_id="odom";
		map_pub.publish(map);
    
}



void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);


	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
	undistImg->timestamp=img->header.stamp.toSec(); // relay the timestamp to HSLAM
	fullSystem->addActiveFrame(undistImg, frameID, filtered_pose);
	
	frameID++;
	if (frameID>5)
	{
	publishResults();
	}
	
	delete undistImg;

}



//NA: Adding interruption code
bool interrupted = false;
void interruptHandler(int signal)
{
	    interrupted = true;
}

void odomFilteredCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Process the received odometry message
	// Create a PoseStamped message from the odometry message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    // Add the new pose to the path
    path_filtered.poses.push_back(pose_stamped);

    // Update the header stamp of the path
    path_filtered.header.stamp = ros::Time::now();

	// Extract the pose from the odometry message
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d t(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
    SE3 filtered_pose(q, t);
	//Write results to files
	std::ofstream outfile;
	outfile.open("/home/nykvm/hslam_ros_ws/results/filtered_result.txt", std::ios_base::app);
	outfile << pose_stamped.header.stamp << " " << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z()<< " " << q.w() << std::endl;
	outfile.close();

    // Publish the path
    // path_filtered_pub.publish(path_filtered);
}


//boost exit handler to exit all threads
void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	while(true) pause();
}


int main( int argc, char** argv )
{		
	boost::thread exThread = boost::thread(exitThread); // hook crtl+C.
	ros::init(argc, argv, "hslam_live");

	for(int i=1; i<argc;i++) parseArgument(argv[i]);


	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;



    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());


    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;
	
	
	IOWrap::PangolinDSOViewer* viewer = 0;
	if(!disableAllDisplay)
    {
        viewer = new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]);
        fullSystem->outputWrapper.push_back(viewer);
    }

    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
	//ros::Rate loop_rate(10);
    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
	// Initialize subscriber
    odom_filtered_sub = nh.subscribe("/odometry/filtered_global", 10, odomFilteredCallback);
	map_pub = nh.advertise<sensor_msgs::PointCloud2>("/hslam/map", 20);
	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/hslam/pose", 20);
	path_pub = nh.advertise<nav_msgs::Path>("/hslam/path", 20);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/hslam/odom", 20);

	path_filtered_pub = nh.advertise<nav_msgs::Path>("/hslam/path_filtered", 20);
	 // Initialize path_filtered
    path_filtered.header.frame_id = "odom";



    //NA: replacing ros_spin with interruptable sequence
    //ros::spin();
	
    signal(SIGINT, interruptHandler);

	while (ros::ok() && !interrupted) //&& frameID <999999 NA
		{
			//printf("ROS IS OKAY!");
			//printf("FrameID: %d ",frameID);
			ros::spinOnce();
			//loop_rate.sleep();
			if(viewer!=0 && viewer->isDead)
					break;
			
			if(fullSystem->isLost)
            {
                printf("LOST!!\n");
                break;
            }
			
			
			if(fullSystem->initFailed || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
				std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
				for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
				usleep(20000); //hack - wait for display wrapper to clean up.
				if(fullSystem)
				{
					delete fullSystem;
					fullSystem = nullptr;
				}
					
				fullSystem = new FullSystem();
    			fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
				fullSystem->linearizeOperation = false;

				fullSystem->outputWrapper = wraps;

				setting_fullResetRequested=false;
            }
			
			
		}
	fullSystem->blockUntilMappingIsFinished();

	printf("hslam_ros main cpp has been interuppted.\n"); //debug NA
	ros::shutdown();
	ros::waitForShutdown();
	fullSystem->BAatExit();
			
	
	std::string resultFilePath = "/home/nykvm/hslam_ros_ws/results/result.txt";
	fullSystem->printResult(resultFilePath.c_str()); 
	fullSystem->saveMap("map.pcd"); 

	//if(viewer != 0)
	//    viewer->run();
	//Clean-up and exit
    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
		//printf("DELETE VIEWER IO wrapper\n");
        ow->join();
        delete ow;
    }

	printf("DELETE FULLSYSTEM!\n");
	delete fullSystem;
	printf("DELETE Undistorter\n");
	delete undistorter;
	printf("EXIT NOW\n");
	return 0;
}

