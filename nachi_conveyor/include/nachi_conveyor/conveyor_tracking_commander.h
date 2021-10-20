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

namespace conveyor_tracking_ns{

class ConveyorTrackingCommander {
public:
    ConveyorTrackingCommander(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~ConveyorTrackingCommander();
    // void init();
    // void update(const ros::TimerEvent& e);
    // void read();
    // void write();
    void test();
    void goPikcing(const nachi_conveyor::SegmentationInfo::ConstPtr& msg);
private:
    ros::ServiceClient conveyorRegistorReadClient;
    ros::ServiceClient conveyorTrackingControlClient;
    //*getShiftPose: return the desired ShiftPost[6] according to the Registor Value, GraspTimes, The offset value of the object in the camera coordinate system: float[7]={X,Y,Z,RX,RY,RZ,RW}
    bool getShiftPose(float,int,float TargetPose[7],float,int);
    bool PoseCheck(float[6]);//Check if the object is graspable;
    bool LockLimitCalculater(float[6]);//calculate the upperlimit and lower limit for calling the conveyorTrackingControl service, return int Limit[upperLimit,lowerLimit];
    float ShiftPoseTemp[6];
    int LimitTemp[2];
    float SumRead;//The sum of historical Read Registor's everytime the registor's value turn to zero;
    float SumValue;//The value SumRead when detact the target object; 
    // const int lengthofGraspFiles = 100;
    float GraspFiles[10000][4];//The length of GraspFiles is set as 100
    // ros_param conveyor_id:
    //service name 
    std::string conveyor_id;
    // ros_param conveyor_tracking
    std::string conveyor_tracking;
    //Transform matrix between camera and robot:
    // float TransformMatrix[4][4];
    // ros::NodeHandle nh_;
    // bool initConnect();
    // ros::Subscriber GraspInfoSubscriber;
    // ros::Publisher statusPublisher;
    // ros::ServiceServer todoService;
    // ros::ServiceClient runningStatusClient;
    // ros::ServiceClient runningStatusClient;
    // ros::ServiceClient callConveyorTracking;
    // float TransformMatrix[4][4] = {{0.99998534,  0.00522063, -0.00143516,  0.373978},
    //                                 {0.00524358, -0.9998503 ,  0.01648864,  1.4090741 },
    //                                 {-0.00134886, -0.01649593, -0.99986302,  0.873961  },
    //                                 {0.        ,  0.        ,  0.        ,  1.        }}; //T(c,r);
    float TransformMatrix[4][4] = {{0.99998534,  0.00522063, -0.00143516,  0.373978},
                                    {0.00524358, -0.9998503 ,  0.01648864,  1.4260741 },
                                    {-0.00134886, -0.01649593, -0.99986302,  0.873961  },
                                    {0.        ,  0.        ,  0.        ,  1.        }}; //T(c,r);
    float PointPPai[6] = {331.73,600,207.36,0.01,0.07,180.00};//just the different of Y;
    float PointP[6]={331.73,0.01,207.36,0.01,0.07,180.00};
    float PointPPaiCposition[4] = {-0.0147391594946, -0.035098630935, 0,1};//the Point PPai's position in camera zuobiaoxi;

    float lTerminal;//It depends on the distense between P and PPai in Y- direction of robot coordinate;
    float WorkSpaceLimitY;//It depends on where you put the Point P, in default, we put the Point P with y=0.0 with robot coordinate;
    float WorkSpaceLimitX1;
    float WorkSpaceLimitX2;
    float WorkSpaceLimitZ1;
    float WorkSpaceLimitZ2;
    float AdjustZ;
    int FollowLength;

  protected:
    ros::NodeHandle &nh_;
    ros::NodeHandle &pnh_;

    ros::Timer update_loop_;
    ros::Timer clock_loop_;
};

}

#endif
