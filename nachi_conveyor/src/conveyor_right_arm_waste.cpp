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
    ros::init(argc,argv,"conveyor_right_arm_grasp");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string mode = "robot1";
    conveyor_tracking_ns::ConveyorTrackingCommander right_commander(nh, pnh, mode);
    right_commander.test();
    ros::Subscriber sub = nh.subscribe("/sorting_line/robot1",20,&conveyor_tracking_ns::ConveyorTrackingCommander::goPikcing,&right_commander);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;

}