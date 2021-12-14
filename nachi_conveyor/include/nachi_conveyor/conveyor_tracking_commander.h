//
// Created by Ray in SMART Lab on 2021/10/15.
//

#ifndef _CONVEYOR_TRACKING_H
#define _CONVEYOR_TRACKING_H

#include <ros/ros.h>
#include "nachi_conveyor/Segmentation.h"
#include "nachi_conveyor/nachiConveyorRegister.h"
#include "nachi_conveyor/nachiConveyorTracking.h"
#include "nachi_conveyor/SegmentationInfo.h"
#include <Eigen/Dense>
#include <list>
// using namespace std;

namespace conveyor_tracking_ns{

class ConveyorTrackingCommander {
public:
    ConveyorTrackingCommander(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string& mode);
    ~ConveyorTrackingCommander();
    void init();
    void test();
    void goPikcing(const nachi_conveyor::SegmentationInfo::ConstPtr& msg);
private:
    ros::ServiceClient conveyorRegistorReadClient;
    ros::ServiceClient conveyorTrackingControlClient;
    bool getShiftPose(float currentRegisterValue, double targetObjectPose[7], float targetRegisterValue);
    bool PoseCheck(float calculatedShiftPose[6]);
    bool LockLimitCalculater(float checkedShiftPose[6]);
    float ShiftPoseTemp_[6];
    int LimitTemp_[2];
    //service name 
    std::string conveyor_registor_value_id_;
    // ros_param conveyor_tracking
    std::string conveyor_tracking_srv_;
    //Transform matrix between camera and robot:
    Eigen::Matrix<float, 4, 4> robot_to_camera_Transform_;
    float base_target_point_under_camera_frame_[4];//the Point PPai's position in camera zuobiaoxi;

    float lTerminal_;//It depends on the distense between P and PPai in Y- direction of robot coordinate;
    float WorkSpaceLimitY_;//It depends on where you put the Point P, in default, we put the Point P with y=0.0 with robot coordinate;
    float WorkSpaceLimitY2;//It depends on where you put the Point P, in default, we put the Point P with y=0.0 with robot coordinate;
    float shift_limit_x_low_;
    float shift_limit_x_upper_;
    float shift_limit_z_low_;
    float shift_limit_z_upper_;
    float shift_pose_adjust_z_;
    float shift_pose_adjust_y_;
    float shift_pose_adjust_x_;
    int follow_object_length_;
    float current_register_value_;
    float register_reset_flag_;

  protected:
    ros::NodeHandle &nh_;
    ros::NodeHandle &pnh_;
    std::string mode_;
};

}

#endif
