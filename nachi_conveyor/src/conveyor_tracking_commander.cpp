//
// Created by Ray in SMART Lab on 2021/10/15.
//

#include "ros/ros.h"
#include "nachi_conveyor/conveyor_tracking_commander.h"
#include "nachi_conveyor/Segmentation.h"
#include "nachi_conveyor/nachiConveyorRegister.h"
#include "nachi_conveyor/nachiConveyorTracking.h"
#include "nachi_conveyor/SegmentationInfo.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <time.h>
const float radiToDegree = 180 / 3.1415926;

namespace conveyor_tracking_ns
{
    ConveyorTrackingCommander::ConveyorTrackingCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& mode) : 
    nh_(nh), pnh_(pnh), mode_(mode)
    {
        init();
    }

    ConveyorTrackingCommander::~ConveyorTrackingCommander()
    {

    }

    void ConveyorTrackingCommander::init()
    {
        pnh_.getParam("lTerminal", lTerminal_);
        pnh_.getParam("WorkSpaceLimitY", WorkSpaceLimitY_);
        pnh_.getParam("WorkSpaceLimitY2", WorkSpaceLimitY2);
        pnh_.getParam("WorkSpaceLimitX1", shift_limit_x_low_);
        pnh_.getParam("WorkSpaceLimitX2", shift_limit_x_upper_);
        pnh_.getParam("WorkSpaceLimitZ1", shift_limit_z_low_);
        pnh_.getParam("WorkSpaceLimitZ2", shift_limit_z_upper_);
        pnh_.getParam("FollowLength", follow_object_length_);
        pnh_.getParam("AdjustZ", shift_pose_adjust_z_);
        pnh_.getParam("AdjustY", shift_pose_adjust_y_);
        pnh_.getParam("AdjustX", shift_pose_adjust_x_);
        pnh_.getParam("conveyor_id", conveyor_registor_value_id_);
        pnh_.getParam("conveyor_tracking", conveyor_tracking_srv_);
        pnh_.getParam("RegisterReset", register_reset_flag_);
        pnh_.getParam("TMatrix00", robot_to_camera_Transform_(0, 0));
        pnh_.getParam("TMatrix01", robot_to_camera_Transform_(0, 1));
        pnh_.getParam("TMatrix02", robot_to_camera_Transform_(0, 2));
        pnh_.getParam("TMatrix03", robot_to_camera_Transform_(0, 3));
        pnh_.getParam("TMatrix10", robot_to_camera_Transform_(1, 0));
        pnh_.getParam("TMatrix11", robot_to_camera_Transform_(1, 1));
        pnh_.getParam("TMatrix12", robot_to_camera_Transform_(1, 2));
        pnh_.getParam("TMatrix13", robot_to_camera_Transform_(1, 3));
        pnh_.getParam("TMatrix20", robot_to_camera_Transform_(2, 0));
        pnh_.getParam("TMatrix21", robot_to_camera_Transform_(2, 1));
        pnh_.getParam("TMatrix22", robot_to_camera_Transform_(2, 2));
        pnh_.getParam("TMatrix23", robot_to_camera_Transform_(2, 3));
        pnh_.getParam("TMatrix30", robot_to_camera_Transform_(3, 0));
        pnh_.getParam("TMatrix31", robot_to_camera_Transform_(3, 1));
        pnh_.getParam("TMatrix32", robot_to_camera_Transform_(3, 2));
        pnh_.getParam("TMatrix33", robot_to_camera_Transform_(3, 3));
        pnh_.getParam("PPaiX", base_target_point_under_camera_frame_[0]);
        pnh_.getParam("PPaiY", base_target_point_under_camera_frame_[1]);
        pnh_.getParam("PPaiZ", base_target_point_under_camera_frame_[2]);
        pnh_.getParam("PPaiXYZ", base_target_point_under_camera_frame_[3]);
        conveyorRegistorReadClient = nh_.serviceClient<nachi_conveyor::nachiConveyorRegister>(conveyor_registor_value_id_);
        conveyorTrackingControlClient = nh_.serviceClient<nachi_conveyor::nachiConveyorTracking>(conveyor_tracking_srv_);
        // ROS_INFO("X:%.4f,Y:%.4f,Z:%.4f,XYZ:%.4f",PointPPaiCposition[0],PointPPaiCposition[1],PointPPaiCposition[2],PointPPaiCposition[3]);
    }

    bool ConveyorTrackingCommander::getShiftPose(float currentRegisterValue, double targetObjectPose[7], float targetRegisterValue)
    {
        if (currentRegisterValue < 0 || targetRegisterValue < 0 || targetObjectPose[2] < 0.2) //TODO All the wrong conditions
        {
            ROS_ERROR("Received invalid target pose, won't calculate shift pose!!!");
            return false;
        }else{
            float DeltaYTemp2 = currentRegisterValue-targetRegisterValue;
            ROS_INFO("The difference between currentRegisterValue and targetRegisterValue is %.4f",DeltaYTemp2);
            Eigen::Matrix<float, 4, 1> ShiftC,ShiftTemp;
            ShiftC << targetObjectPose[0] - base_target_point_under_camera_frame_[0],
                targetObjectPose[1] - base_target_point_under_camera_frame_[1],
                targetObjectPose[2] - base_target_point_under_camera_frame_[2],
                0.0;
            ShiftTemp = (robot_to_camera_Transform_ * ShiftC)*1000;
            ROS_INFO("The value of base Point is:x:.%9f,y:.%9f,.z:.%9f",base_target_point_under_camera_frame_[0],base_target_point_under_camera_frame_[1],base_target_point_under_camera_frame_[2]);
            ROS_INFO("The receive point is: X:%.9f, Y:%.9f, Z:%.9f", targetObjectPose[0], targetObjectPose[1], targetObjectPose[2]);
            ROS_INFO("The calculated shift pose between target object and camera base point is: DeltaX:%.2f, DeltaY:%.2f",ShiftTemp(0),ShiftTemp(1));
            ShiftPoseTemp_[0]=ShiftTemp(0)+shift_pose_adjust_x_;
            // ShiftPoseTemp_[1]=ShiftTemp(1)+DeltaYTemp2+shift_pose_adjust_y_;//DeltaYTemp1
            ShiftPoseTemp_[1]=DeltaYTemp2-ShiftTemp(1);//DeltaYTemp，Here attention!!! This value is length in Y- direction of robot coordinate of target that shift from target PointPPaiCposition, not the real value that will be sent to the robot, the real value will be update when calculate the LockLimit in LockLimitCalculater!!!
            ROS_INFO("Y value for check is:%.3f",ShiftPoseTemp_[1]);
            ShiftPoseTemp_[2]=shift_pose_adjust_z_;
            ROS_INFO("Finishing calcuting the ShiftPose!!");
            ShiftPoseTemp_[3]=0;
            ShiftPoseTemp_[4]=0;
            //Robot with sucking cup
            if (mode_=="robot2")
            {
                ShiftPoseTemp_[5] = 0;
            }
            //Robot with gripper
            if (mode_=="robot1")
            {
                float rotation_angle_z;
                rotation_angle_z = 2 * asin(targetObjectPose[5]);
                ROS_INFO("The degree in z axis for rotation is:.%2f",rotation_angle_z*radiToDegree);
                if (rotation_angle_z>=0)
                {
                    ShiftPoseTemp_[5] = 90 - radiToDegree*rotation_angle_z;                   
                }
                else
                {
                    ShiftPoseTemp_[5] = -90 - rotation_angle_z*radiToDegree;
                }
                ROS_INFO("The RZ send to the robot is:.%2f",ShiftPoseTemp_[5]);
                if (ShiftPoseTemp_[5]>45)
                {
                    ShiftPoseTemp_[5] = 45;
                }
                if (ShiftPoseTemp_[5]<-45)
                {
                    ShiftPoseTemp_[5] = -45;
                }
                                               
                /* code */
            }
            return true;
        }
    }

    bool ConveyorTrackingCommander::LockLimitCalculater(float checkedShiftPose[6])
    {
        if (checkedShiftPose[0]<-100000)//TODO All the wrong conditions
        {
            return false;
            /* code */
        }else{
            float executeRegisterValue = 0;
            if (current_register_value_<register_reset_flag_)
            {
                executeRegisterValue = current_register_value_;
                    /* code */
            }
            if (checkedShiftPose[1]<lTerminal_-WorkSpaceLimitY_)
            {
                ROS_WARN("Waiting mode!");
                LimitTemp_[0] = executeRegisterValue+lTerminal_-WorkSpaceLimitY_-checkedShiftPose[1];
                LimitTemp_[1] = LimitTemp_[0] + follow_object_length_;
                ShiftPoseTemp_[1] = LimitTemp_[0]-lTerminal_+WorkSpaceLimitY_+shift_pose_adjust_y_;
                ROS_WARN("4.2) The Limits are set as:");
                ROS_WARN("The upper Limit is:%d",LimitTemp_[0]);
                ROS_WARN("The lower Limit is:%d",LimitTemp_[1]);
                ROS_WARN("4.3) The calculated ShiftPose for robot are x:.%2f,y:.%2f,z:.%2f,RZ:.%2f",ShiftPoseTemp_[0],ShiftPoseTemp_[1],ShiftPoseTemp_[2],ShiftPoseTemp_[5]);
                /* code */
            }else{
                ROS_WARN("Meeting point mode!");
                LimitTemp_[0] = executeRegisterValue;
                LimitTemp_[1] = LimitTemp_[0] + follow_object_length_;
                ShiftPoseTemp_[1] = LimitTemp_[0]-checkedShiftPose[1]+shift_pose_adjust_y_;
                ROS_WARN("4.2) The Limits are set as:");
                ROS_WARN("The upper Limit is:%d", LimitTemp_[0]);
                ROS_WARN("The lower Limit is:%d", LimitTemp_[1]);
                ROS_WARN("4.3) The calculated ShiftPose for robot are x:.%2f,y:.%2f,z:.%2f,RZ:.%2f", ShiftPoseTemp_[0], ShiftPoseTemp_[1], ShiftPoseTemp_[2], ShiftPoseTemp_[5]);
            }
            return true;
        }
    }

    bool ConveyorTrackingCommander::PoseCheck(float calculatedShiftPose[6])
    {
        float ConvayorDirectionSpaceLimit; //-250is the limit in Y+ direction for robot when it located in the PPosition
        ConvayorDirectionSpaceLimit = lTerminal_ + WorkSpaceLimitY_ - follow_object_length_;
        if (calculatedShiftPose[0]<=shift_limit_x_low_||calculatedShiftPose[0]>=shift_limit_x_upper_||calculatedShiftPose[1]<=WorkSpaceLimitY2||calculatedShiftPose[1]>=ConvayorDirectionSpaceLimit||calculatedShiftPose[2]<=shift_limit_z_low_||calculatedShiftPose[2]>=shift_limit_z_upper_)
        {
            ROS_WARN("Wrong Pose, Won't do anything!!!");
            return false;
        }else{
            ROS_INFO("Right Pose, pose check finished!");
            return true;
        }
    }

    void ConveyorTrackingCommander::test()
    {
    ROS_INFO("Hello guys, Welcome to SMART Lab Aluminium project, GoodLuck");
    }
    void ConveyorTrackingCommander::goPikcing(const nachi_conveyor::SegmentationInfo::ConstPtr& segInfo)
    {
        ROS_INFO("================================================================");
        ROS_INFO("Start deciding if the robot should go to pick or not!!!");
        // clock_t startTime, endTime;
        // startTime = clock();
        nachi_conveyor::nachiConveyorRegister registerRead;
        registerRead.request.Get_Conveyor_value = true;//
        conveyorRegistorReadClient.call(registerRead);
        //get the status of ConveyorRegistorValueserver,
        ros::service::waitForService(conveyor_registor_value_id_);
        bool register_value_srv_call_flag = conveyorRegistorReadClient.call(registerRead);
        if (register_value_srv_call_flag)
        {
            ROS_INFO("1) Get the Current Conveyor Registor Read value is: %.2f",registerRead.response.Register_value);
            current_register_value_ = registerRead.response.Register_value;
            double TargetPoseTemp[7] = {segInfo->object_pose.pose.position.x, segInfo->object_pose.pose.position.y, segInfo->object_pose.pose.position.z, segInfo->object_pose.pose.orientation.x, segInfo->object_pose.pose.orientation.y, segInfo->object_pose.pose.orientation.z, segInfo->object_pose.pose.orientation.z};

            bool shift_pose_calculate_flag = getShiftPose(registerRead.response.Register_value, TargetPoseTemp, segInfo->Register_value);

            ROS_INFO("3) The calculated ShiftPose for chack is x:.%2f,y:.%2f,z:.%2f,RZ:.%2f", ShiftPoseTemp_[0], ShiftPoseTemp_[1], ShiftPoseTemp_[2], ShiftPoseTemp_[5]);

            bool target_is_graspable_flag = PoseCheck(ShiftPoseTemp_);

            if (shift_pose_calculate_flag && target_is_graspable_flag)
            {
                ROS_INFO("4) PoseCheck, Right Pose, now going to pick the target!!!");
                //3) use the ShiftPoseTemp_ to initialize the ConveyorTrackingsrv
                //3.1) call the LockLimitCalculater function to calculate the upperLimit and lowerLimit:
                int upperLimit, lowerLimit;
                if (LockLimitCalculater(ShiftPoseTemp_))
                {
                    upperLimit = LimitTemp_[0];
                    lowerLimit = LimitTemp_[1];
                    //3.2) create a ConveyorTrackingsrv and initialize it with the calculated Limits and ShiftPose:
                    nachi_conveyor::nachiConveyorTracking ConveyorTrackingsrv;
                    //initialize the srv.request values:
                    ConveyorTrackingsrv.request.upper_limit = upperLimit;
                    ConveyorTrackingsrv.request.lower_limit = lowerLimit;
                    if (current_register_value_ > register_reset_flag_)
                    {
                        ConveyorTrackingsrv.request.ifRegisterReset = 0;
                        /* code */
                    }
                    else
                    {
                        ConveyorTrackingsrv.request.ifRegisterReset = 2;
                    }
                    ConveyorTrackingsrv.request.shift_pose = {ShiftPoseTemp_[0], ShiftPoseTemp_[1], ShiftPoseTemp_[2], ShiftPoseTemp_[3], ShiftPoseTemp_[4], ShiftPoseTemp_[5]};
                    //  classes: ['background', 'wire', 'can', 'stainless steel', 'copper chunk', 'aluminium chunk', 'foam', 'pcb', 'battery', 'useless waste']
                    if (mode_ == "robot2")
                    {
                        int type;
                        if (strcmp(segInfo->class_name.c_str(), "pcb") == 0)
                        {

                            type = 1;
                            /* code */
                        }
                        if (strcmp(segInfo->class_name.c_str(), "can") == 0)
                        {
                            type = 2;
                            /* code */
                        }
                        ConveyorTrackingsrv.request.material_Type = type;
                        /* code */
                    }
                    if (mode_ == "robot1")
                    {
                        int type;
                        if (strcmp(segInfo->class_name.c_str(), "wire") == 0)
                        {

                            type = 1;
                            /* code */
                        }
                        if (strcmp(segInfo->class_name.c_str(), "copper chunk") == 0)
                        {
                            type = 2;
                            /* code */
                        }
                        ConveyorTrackingsrv.request.material_Type = type;
                        /* code */
                    }
                    //4) call the conveyorTrackingControl
                    ros::service::waitForService(conveyor_tracking_srv_);
                    // endTime = clock();
                    // double DeltaTime = double(endTime - startTime);
                    // ROS_ERROR("The total time used by calculting is: %.2f",DeltaTime);
                    bool conveyor_tracking_srv_call_flag = conveyorTrackingControlClient.call(ConveyorTrackingsrv);
                    if (conveyor_tracking_srv_call_flag)
                    {
                        ROS_INFO("That's right!");
                        ROS_INFO("The robot program is about to finish running!!!");
                        ROS_INFO("Finish Grasp ");
                        /* code */
                    }
                    else
                    {
                        ROS_ERROR("Something goes wrong in the Robot side, Robot won't run!!!");
                    }
                    // ROS_ERROR("Tracking control callback finish, task %d finished!!", currentGraspTimes+1);
                    ROS_INFO("Tracking control callback finish, task finished!!");
                    ROS_INFO("================================================================");
                    /* code */
                }
                else
                {
                    ROS_ERROR("Limit calculate Wrong!!! The Limt value is Wrong, so the limit is set as a very large number!!!!!!!!!!!!!!!!");
                }
            }
            else
            { //Pose Check Wrong:
                ROS_WARN("Target Pose is invalid!!! Or the calculated ShiftPose is invalid!! Task finished!!");
                ROS_INFO("================================================================");
            }
        }else{
            ROS_ERROR("Something goes wrong, you won't get the currentRegistorValue, Please check the libnachi node!!!");
        }
    }
}