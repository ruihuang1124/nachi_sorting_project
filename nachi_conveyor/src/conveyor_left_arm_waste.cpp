//
// Created by Ray in SMART Lab on 2021/10/15.
//

#include "ros/ros.h"
#include "nachi_conveyor/Segmentation.h"
#include "nachi_conveyor/nachiConveyorRegister.h"
#include "nachi_conveyor/nachiConveyorTracking.h"
#include "nachi_conveyor/SegmentationInfo.h"
#include "nachi_conveyor/conveyor_tracking_commander.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"conveyor_left_arm_grasp");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string mode = "robot2";
    conveyor_tracking_ns::ConveyorTrackingCommander left_commander(nh, pnh, mode);
    left_commander.test();
    ros::Subscriber sub = nh.subscribe("/sorting_line/robot2",20,&conveyor_tracking_ns::ConveyorTrackingCommander::goPikcing,&left_commander);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;

}