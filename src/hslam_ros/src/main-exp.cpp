/**
 * This file is part of FLSAM_ROS.
 * Based on and inspired by DSO project by Jakob Engel
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
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace HSLAM;

// Encapsulate system state in a struct
struct HSLAMSystemState {
    FullSystem* fullSystem = nullptr;
    Undistort* undistorter = nullptr;
    int frameID = 0;
    bool interrupted = false;
} hslamState;

ros::Publisher map_pub;
ros::Publisher pose_pub;
ros::Publisher path_pub;

void parseArgument(char* arg);
void publishResults();
void vidCb(const sensor_msgs::ImageConstPtr img);
void resetSystem(HSLAMSystemState& state);
//Arguments Initialization
std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string saveFile = "";
std::string vocabPath = "";
bool useSampleOutput=false;
int mode = 0;
int preset= 0;
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

void publishResults() {        
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose_stamped;
    sensor_msgs::PointCloud2 map;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::vector<SE3> points = hslamState.fullSystem->getPath();
    std::vector<Eigen::Vector3f> map_points = hslamState.fullSystem->getMap();

    path.header.frame_id = "map";
    pose_stamped.header.frame_id = "map";

    for (const auto& point : points) {
        pose_stamped.pose.position.x = point.translation().x();
        pose_stamped.pose.position.y = point.translation().y();
        pose_stamped.pose.position.z = point.translation().z();
        pose_stamped.pose.orientation.x = point.so3().unit_quaternion().x();
        pose_stamped.pose.orientation.y = point.so3().unit_quaternion().y();
        pose_stamped.pose.orientation.z = point.so3().unit_quaternion().z();
        pose_stamped.pose.orientation.w = point.so3().unit_quaternion().w();
        path.poses.push_back(pose_stamped);
    }

    path_pub.publish(path);
    pose_pub.publish(pose_stamped);

    for (const auto& map_point : map_points) {
        pcl::PointXYZ point;
        point.x = map_point.x();
        point.y = map_point.y();
        point.z = map_point.z();
        cloud.push_back(point);
    }
    pcl::toROSMsg(cloud, map);
    map.header.frame_id = "map";
    map_pub.publish(map);
}

void vidCb(const sensor_msgs::ImageConstPtr img) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    if (setting_fullResetRequested) {
        resetSystem(hslamState);
    }

    MinimalImageB minImg(cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.data);
    ImageAndExposure* undistImg = hslamState.undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);
    undistImg->timestamp = img->header.stamp.toSec(); // relay the timestamp to HSLAM

    hslamState.fullSystem->addActiveFrame(undistImg, hslamState.frameID++);
    if (hslamState.frameID > 50) {
        publishResults();
    }
    
    delete undistImg;
}

void resetSystem(HSLAMSystemState& state) {
    std::vector<IOWrap::Output3DWrapper*> wraps = state.fullSystem->outputWrapper;
    delete state.fullSystem;
    for (IOWrap::Output3DWrapper* ow : wraps) ow->reset();

    state.fullSystem = new FullSystem();
    state.fullSystem->linearizeOperation = false;
    state.fullSystem->outputWrapper = wraps;

    if (state.undistorter->photometricUndist != nullptr)
        state.fullSystem->setGammaFunction(state.undistorter->photometricUndist->getG());
    
    setting_fullResetRequested = false;
}

int main(int argc, char** argv) {    
    ros::init(argc, argv, "hslam_live");

    for (int i = 1; i < argc; i++) parseArgument(argv[i]);

    hslamState.undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
        (int)hslamState.undistorter->getSize()[0],
        (int)hslamState.undistorter->getSize()[1],
        hslamState.undistorter->getK().cast<float>()
    );

    hslamState.fullSystem = new FullSystem();
    hslamState.fullSystem->linearizeOperation = false;

    IOWrap::PangolinDSOViewer* viewer = nullptr;
    if (!disableAllDisplay) {
        viewer = new IOWrap::PangolinDSOViewer(
            (int)hslamState.undistorter->getSize()[0],
            (int)hslamState.undistorter->getSize()[1]
        );
        hslamState.fullSystem->outputWrapper.push_back(viewer);
    }

    if (useSampleOutput) {
        hslamState.fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
    }

    if (hslamState.undistorter->photometricUndist != nullptr) {
        hslamState.fullSystem->setGammaFunction(hslamState.undistorter->photometricUndist->getG());
    }

    ros::NodeHandle nh;
    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/hslam/map", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hslam/pose", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/hslam/path", 10);

    ros::Rate loop_rate(20);  // Adjust the rate as needed

    while (ros::ok() && !hslamState.interrupted) {
        ros::spinOnce();
        loop_rate.sleep();

        if (viewer != nullptr && viewer->isDead) break;
        if (hslamState.fullSystem->isLost) {
            ROS_WARN("System is lost!");
            break;
        }

        if (hslamState.fullSystem->initFailed || setting_fullResetRequested) {
            ROS_INFO("System resetting!");
            resetSystem(hslamState);
        }
    }

    hslamState.fullSystem->blockUntilMappingIsFinished();

    ROS_INFO("Shutting down...");
    ros::shutdown();
    hslamState.fullSystem->BAatExit();
    hslamState.fullSystem->printResult("result.txt");
    hslamState.fullSystem->saveMap("map.pcd");

    for (IOWrap::Output3DWrapper* ow : hslamState.fullSystem->outputWrapper) {
        ow->join();
        delete ow;
    }

    delete hslamState.fullSystem;
    delete hslamState.undistorter;

    return 0;
}
