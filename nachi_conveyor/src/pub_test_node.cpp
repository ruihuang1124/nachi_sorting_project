#include "ros/ros.h"
#include "nachi_conveyor/Segmentation.h"
#include "nachi_conveyor/nachiConveyorRegister.h"
#include "nachi_conveyor/nachiConveyorTracking.h"
#include "nachi_conveyor/SegmentationInfo.h"
#include "nachi_conveyor/conveyor_tracking_commander.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"SegmentationInfo_pub_test_node");
    //create ros nodehandel:
    ros::NodeHandle nh;
    
    //4  create pub object
    ros::Publisher pub = nh.advertise<nachi_conveyor::SegmentationInfo>("/left_arm/Seg_Info_test",10);
    //5 write pub logic and pub msg
    // create the pub msg
    int detectGraspTimes=0;
    bool result = nh.getParam("/left_arm_grasp_times",detectGraspTimes);
    if (!result)
    {
        ROS_ERROR("no /left_arm_grasp_times");
        /* code */
    }
    
    nachi_conveyor::SegmentationInfo segInfo;
    segInfo.object_pose.header.seq = 1;
    segInfo.object_pose.header.stamp = ros::Time::now();
    segInfo.object_pose.header.frame_id = "frame_1";
    segInfo.object_pose.pose.position.x = 0;
    segInfo.object_pose.pose.position.y = 0;
    segInfo.object_pose.pose.position.z = 0;
    segInfo.object_pose.pose.orientation.x = 0;
    segInfo.object_pose.pose.orientation.y = 0;
    segInfo.object_pose.pose.orientation.z = 0;
    segInfo.object_pose.pose.orientation.w = 0;
    segInfo.class_name = "test1";
    segInfo.Register_value = 600;
    segInfo.GraspTimes = detectGraspTimes;
    //set pub rate
    ros::Rate rate(0.05);
    int a = 100;
    int b = 100;
    int c = a%b;
    //float testFiles[100][2];
    //pub through loop
    while (ros::ok())
    {
        ROS_ERROR("The test value is %d",c);
        //change the date
        nh.getParam("/left_arm_grasp_times",detectGraspTimes);
        segInfo.GraspTimes = detectGraspTimes;
        segInfo.object_pose.header.seq += 1;
        // main task: pub the msg
        pub.publish(segInfo);
        //ROS_INFO print:
        ROS_INFO("Publish the message: seq: %d, Register_value: %.2f, detect_Grasp_times:%d",segInfo.object_pose.header.seq,segInfo.Register_value,segInfo.GraspTimes);
        //sleep
        rate.sleep();
        //suggess to add:
        ros::spinOnce();
        /* code */
    }

    return 0;

}