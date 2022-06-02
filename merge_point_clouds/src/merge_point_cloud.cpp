#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Imu.h>
#include<pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include<pcl_conversions/pcl_conversions.h>


ros::Publisher pub ; 
tf::TransformListener *listener_ns1, *listener_ns2;
tf::StampedTransform transform;


void callback (const sensor_msgs::PointCloud2ConstPtr& ns1 , const sensor_msgs::PointCloud2ConstPtr& ns2)
{
    sensor_msgs::PointCloud2 ns1_tf, ns2_tf, merged_ns1;
    listener_ns1->waitForTransform("/vlp16_merged", "/vlp16_port", (*ns1).header.stamp , ros::Duration(5.0));
    listener_ns1->lookupTransform("/vlp16_merged", "/vlp16_port", (*ns1).header.stamp, transform); //ros::Time(0)

    pcl_ros::transformPointCloud("/vlp16_merged", *ns1, ns1_tf, *listener_ns1);

    listener_ns2->waitForTransform("/vlp16_merged", "/vlp16_starboard", (*ns2).header.stamp , ros::Duration(5.0));
    listener_ns2->lookupTransform("/vlp16_merged", "/vlp16_starboard", (*ns2).header.stamp, transform); //ros::Time(0)

    pcl_ros::transformPointCloud("/vlp16_merged", *ns2, ns2_tf, *listener_ns2);


    pcl::concatenatePointCloud(ns1_tf, ns2_tf, merged_ns1);
    
    pub.publish(merged_ns1);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_merger");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_velodyne_points", 100);


    message_filters::Subscriber<sensor_msgs::PointCloud2> ns1_sub(nh, "/ns1/velodyne_points", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> ns2_sub(nh, "/ns2/velodyne_points", 100);
  

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ns1_sub, ns2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    listener_ns1 = new tf::TransformListener();
    listener_ns2 = new tf::TransformListener();
    
    ros::spin();
}