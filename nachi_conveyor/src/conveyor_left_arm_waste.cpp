//
// Created by Ray in SMART Lab on 2021/10/15.
//

#include "ros/ros.h"
#include "nachi_conveyor/Segmentation.h"
#include "nachi_conveyor/nachiConveyorRegister.h"
#include "nachi_conveyor/nachiConveyorTracking.h"
#include "nachi_conveyor/SegmentationInfo.h"
#include "nachi_conveyor/conveyor_tracking_commander.h"
/*
namespace: conveyor_tracking_ns 
class: ConveyorTrackingCommander
*/
int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"conveyor_left_arm_grasp");
    //create ros nodehandel:
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //nh.setParam("left_arm_grasp_times",0);
    // nh.setParam("right_arm_grasp_times",0);
    //create sub obj:
    // conveyor_left_ns::ConveyorLeftCommand conveyorLC;
    // ros::Subscriber sub = nh.subscribe("/left_arm/Robot_picking",10,goPicking);
    conveyor_tracking_ns::ConveyorTrackingCommander left_commander(nh, pnh);
    left_commander.test();
    ros::Subscriber sub = nh.subscribe("/left_arm/Seg_Info_test",10,&conveyor_tracking_ns::ConveyorTrackingCommander::goPikcing,&left_commander);
    ros::AsyncSpinner spinner(4);//multi thread spin
    spinner.start();
    ros::waitForShutdown();

    return 0;

}