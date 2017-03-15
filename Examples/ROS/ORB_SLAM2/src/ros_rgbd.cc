/*
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/*
* Pull #102: github.com/raulmur/ORB_SLAM2/pull/102
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TransformStamped.h"
#include "../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

geometry_msgs::PoseStamped msg; /**< Message to be filled with position data */

/* FIXME uncomment to publish the map on a topic
 * typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 * PointCloud::Ptr mapp (new PointCloud);
 * float x, y, z;
 * int npunti;
 */
  
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    // Create the pose publisher    
    ros::Publisher pubs = nh.advertise<geometry_msgs::PoseStamped>("my_pose_stamped", 1)
  
    // FIXME uncomment to publish the map on a topic
    // ros::Publisher pubMap = nh.advertise<PointCloud>("my_pcl_map", 1);
     
    ros::Rate loop_rate(10);
   
    // Subscribing to topics
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/rgb_in", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth_in", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    while(ros::ok()){
        
        pubs.publish(msg); // Publish the pose
    	
	/* FIXME uncomment to publish the map on a topic	  
  	 * if (!(mapp->points.empty())){
         *     mapp->header.stamp = ros::Time::now().toSec();
   	 *     pubMap.publish(mapp);
         * }
         */
       
    	ros::spinOnce();
    	loop_rate.sleep();
    }

    ros::spin();
     
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())
        return;

    // global left handed coordinate system
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);

    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                               -1, 1,-1, 1,
                                                               -1,-1, 1, 1,
                                                                1, 1, 1, 1);

    // prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();


    /* transform into global right handed coordinate system, publish in ROS*/
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                  - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    // Adjust this rotation matrix according to your needs
    const tf::Matrix3x3 rotation270degXZ(   1, 0, 0,
                                            0, 0, 1,
                                            0, -1, 0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_pose"));

     
    // Transform the rotation matrix into a quaternion
    tf::Quaternion tfqt;
    globalRotation_rh.getRotation(tfqt);
    
    // Fill the fileds of the pose message 
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/firefly/base_link";
    msg.pose.position.x = transform.getOrigin().getX();
    msg.pose.position.y = transform.getOrigin().getY();
    msg.pose.position.z = transform.getOrigin().getZ();
    msg.pose.orientation.x = tfqt.x() ;
    msg.pose.orientation.y = tfqt.y() ;
    msg.pose.orientation.z = tfqt.z() ; 
    msg.pose.orientation.w = tfqt.w() ;
    
    /* FIXME uncomment to publish the map on a topic
     * const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpSLAM->mpMap->GetAllMapPoints();
     * const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAM->mpMap->GetReferenceMapPoints();
     * set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
     * mapp->points.clear();
     * if(!vpMPs.empty()){
     *     npunti =0;
     *     for(size_t i=0, iend=vpMPs.size(); i<iend;i++) {
     *         if(vpMPs[i]->isBad()) continue;
     *         cv::Mat pos = vpMPs[i]->GetWorldPos();
     *         x = pos.at<float>(0); 
     *         y = pos.at<float>(1);
     *         z = pos.at<float>(2);
     *         mapp->points.push_back(pcl::PointXYZ(x,y,z));
     *         npunti++; 
     *     }
     *     mapp->header.stamp = ros::Time::now().toSec();
     *     mapp->header.frame_id = "camera_link"; 
     *     mapp->height =1;
     *     mapp->width = npunti;
     * } */
}


