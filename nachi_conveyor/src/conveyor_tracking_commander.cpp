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

//Some default const values:
// const float TestValue = 0.001;


namespace conveyor_tracking_ns
{
    ConveyorTrackingCommander::ConveyorTrackingCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
    nh_(nh), pnh_(pnh)
    {
        // lTerminal = 1460;//It depends on the distense between P and PPai in Y- direction of robot coordinate;
        // WorkSpaceLimitY = 250;//It depends on where you put the Point P, in default, we put the Point P with y=0.0 with robot coordinate;
        // WorkSpaceLimitX1 = -122;
        // WorkSpaceLimitX2 = 80;
        // WorkSpaceLimitZ1 = -10;
        // WorkSpaceLimitZ2 = 100;
        // FollowLength = 100;
        pnh_.getParam("lTerminal", lTerminal);
        pnh_.getParam("WorkSpaceLimitY", WorkSpaceLimitY);
        pnh_.getParam("WorkSpaceLimitX1", WorkSpaceLimitX1);
        pnh_.getParam("WorkSpaceLimitX2", WorkSpaceLimitX2);
        pnh_.getParam("WorkSpaceLimitZ1", WorkSpaceLimitZ1);
        pnh_.getParam("WorkSpaceLimitZ2", WorkSpaceLimitZ2);
        pnh_.getParam("FollowLength", FollowLength);
        pnh_.getParam("AdjustZ", AdjustZ);
        SumRead = 0.0;
        SumValue = 0.0;
        pnh_.getParam("conveyor_id", conveyor_id);
        pnh_.getParam("conveyor_tracking", conveyor_tracking);
        conveyorRegistorReadClient = nh_.serviceClient<nachi_conveyor::nachiConveyorRegister>(conveyor_id);
        conveyorTrackingControlClient = nh_.serviceClient<nachi_conveyor::nachiConveyorTracking>(conveyor_tracking);
    }

    ConveyorTrackingCommander::~ConveyorTrackingCommander()
    {

    }

    bool ConveyorTrackingCommander::getShiftPose(float currentRegisterValue, int currentGraspTimes, float SP[7], float targetRegisterValue, int targetGraspTimes)
    {        
        if (currentRegisterValue<0||currentGraspTimes<0||targetGraspTimes<0||targetRegisterValue<0)//All the wrong conditions, to be added and finished!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            ROS_ERROR("Target Pose is invilid!!!");
            return false;
            /* code */
        }else{
            //1) first calculate the DeltaYTemp2: DeltaY = DeltaYTemp2+DeltaYTemp1;
            float DeltaYTemp2 = 0;
            float SumTemp1=currentRegisterValue;//T(i-1,i)
            float SumTemp2=targetRegisterValue;//M(j,i)
            int listLocation = currentGraspTimes%10000;
            if (listLocation == 0)
            {
                SumRead = 0.0;
                SumValue = 0.0;
                /* code */
            }else{
                SumRead = GraspFiles[listLocation-1][2]+GraspFiles[listLocation-1][0];
                GraspFiles[listLocation][2] = SumRead;//make sure nothing goes wrong when targetGraspTimes == currentGraspTimes, it can update the GraspFile so the SumValue is the right value;
                SumValue = GraspFiles[targetGraspTimes][2];//Some problems here, if the targetGraspTimes =             
            }
            ROS_ERROR("The SumRead value When calculate the shift pose is:.%2f",SumRead);
            ROS_ERROR("The SumValue value When calculate the shift pose is:.%2f",SumValue);
            SumTemp1 = currentRegisterValue + SumRead;
            SumTemp2 = targetRegisterValue + SumValue;
            DeltaYTemp2 = -(SumTemp1 - SumTemp2);//anti (SumTemp1-SumTemp2);
            //2) Then to calculate the DeltaXTemp1,DeltaYTemp1,DeltaZTemp1;
            float DeltaPosition[4] = {SP[0]-PointPPaiCposition[0],SP[1]-PointPPaiCposition[1],SP[2]-PointPPaiCposition[2],1};
            float DeltaXTemp1 = TransformMatrix[0][0]*DeltaPosition[0]+TransformMatrix[0][1]*DeltaPosition[1]+TransformMatrix[0][2]*DeltaPosition[2]+TransformMatrix[0][3]*DeltaPosition[3];
            float DeltaYTemp1 = TransformMatrix[1][0]*DeltaPosition[0]+TransformMatrix[1][1]*DeltaPosition[1]+TransformMatrix[1][2]*DeltaPosition[2]+TransformMatrix[1][3]*DeltaPosition[3];
            float DeltaZTemp1 = TransformMatrix[2][0]*DeltaPosition[0]+TransformMatrix[2][1]*DeltaPosition[1]+TransformMatrix[2][2]*DeltaPosition[2]+TransformMatrix[2][3]*DeltaPosition[3];
            float DeltaXYZTemp1 = TransformMatrix[3][0]*DeltaPosition[0]+TransformMatrix[3][1]*DeltaPosition[1]+TransformMatrix[3][2]*DeltaPosition[2]+TransformMatrix[3][3]*DeltaPosition[3];

            ShiftPoseTemp[0]=DeltaXTemp1;
            ShiftPoseTemp[1]=DeltaYTemp1+DeltaYTemp2;//DeltaYTemp1+DeltaTemp1;
            ShiftPoseTemp[2]=AdjustZ;
            ROS_ERROR("The calcute ShiftPose is x:.%2f,y:.%2f,z:.%2f",ShiftPoseTemp[0],ShiftPoseTemp[1],ShiftPoseTemp[2]);
            ShiftPoseTemp[3]=0;
            ShiftPoseTemp[4]=0;
            ShiftPoseTemp[5]=0;
        // return ShiftPose according to the current currentRegisterValue, GraspTimes and imige's geometry_msgs/PoseStamped object_pose's position and orientation informations.
            return true;
        }
    }

    bool ConveyorTrackingCommander::LockLimitCalculater(float SP[6])
    {
        if (SP[0]<-100000)//All the wrong conditions, to be added and finished!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            return false;
            /* code */
        }else{
            // if (ShiftPoseTemp[1])
            // {
            //     /* code */
            // }
            float decider = lTerminal + SP[1];
            if (decider<WorkSpaceLimitY)
            {
                LimitTemp[0] = 0;
                LimitTemp[1] = FollowLength;
                ROS_ERROR("The upper Limit is:%d",LimitTemp[0]);
                ROS_ERROR("The lower Limit is:%d",LimitTemp[1]);
                /* code */
            }else{
                LimitTemp[0] = int(decider-WorkSpaceLimitY);//Then it will wait untile the object reach the upperLimit
                LimitTemp[1] = int(decider-WorkSpaceLimitY)+FollowLength;
                ROS_ERROR("The upper Limit is:%d",LimitTemp[0]);
                ROS_ERROR("The lower Limit is:%d",LimitTemp[1]);
            }
            
            // LimitTemp[0]=200; //calculate the upper limit
            // LimitTemp[1]=400; //calculate the lower limit
        // return ShiftPose according to the current currentRegisterValue, GraspTimes and imige's geometry_msgs/PoseStamped object_pose's position and orientation informations.
            return true;
        }
        
        // return Limit;
    }

    bool ConveyorTrackingCommander::PoseCheck(float SP[6])
    {
        float tempdecider1 = -lTerminal-WorkSpaceLimitY;//-250is the limit in Y+ direction for robot when it located in the PPosition
        if (SP[0]<=WorkSpaceLimitX1||SP[0]>=WorkSpaceLimitX2||SP[1]<=tempdecider1||SP[1]>=80||SP[2]<=WorkSpaceLimitZ1||SP[2]>=WorkSpaceLimitZ2)
        {
            ROS_ERROR("Wrong Pose, Won't do anything!!!");
            return false;
        }else{
            ROS_ERROR("Right Pose, going to pick it!!!");
            return true;
        }
    }

    void ConveyorTrackingCommander::test()
    {
    ROS_INFO("Hello guys, Welcome to SMART Lab Aluminium project, GoodLuck");
    }
    void ConveyorTrackingCommander::goPikcing(const nachi_conveyor::SegmentationInfo::ConstPtr& segInfo)
    {
        ROS_ERROR("The Goal GraspTimes: %d, Register value:.%2f",segInfo->GraspTimes,segInfo->Register_value);
        ROS_ERROR("Start picking or not!!!");
        //first make sure the current robot program has finished running:


        // ROS_INFO("Receive the message: seq: %d, register value: %.2f", segInfo->object_pose.header.seq,segInfo->Register_value);
        /*
        callBack Function when receive the target segInfo msg:
        1) Call the nachiConveyorRegistor service to get the currentRegister value;

        2) get the currentGraspTimes through rosparam get

        3) Check if the target grasp pose is ok with the: segInfo msg, currentRegistervalue, GraspFiles, currentGraspTimes
            3.1) calculate the ShiftPoseTem according to the currentRegistervalue, Temlist, currentGraspTimes, segInfo msg
            3.2) check if the ShiftPoseTem is OK or not,
                if not ok, finish this call back fun
                if ok: 1) go to pick this object and 2)update some params, e.g. Temlist, left_arm_grasp_times, 
        */


        //1) first to get the conveyor register's value to calculate the goal R1 and lu&ll value:
        // handle and response the srv
        nachi_conveyor::nachiConveyorRegister registerRead;
        registerRead.request.Get_Conveyor_value = true;//
        conveyorRegistorReadClient.call(registerRead);
        //get the status of ConveyorRegistorValueserver,
        //client.waitForExistence();
        //or  we can do this:
        // ROS_INFO("waiting for the ConveyorRegister Service to come up!!!!!");
        ros::service::waitForService(conveyor_id);
        bool flag1 = conveyorRegistorReadClient.call(registerRead);
        if (flag1)
        {
            // ROS_INFO("That's right!");
            ROS_INFO("The Current Conveyor Registor Read value is: %.2f",registerRead.response.Register_value);
            /* code */
        }else{
            ROS_ERROR("Something goes wrong, you won't get the currentRegistorValue!!!");
        }

        //2) get the current grasp times:
        int currentGraspTimes;
        bool result = nh_.getParam("/left_arm_waste/left_arm_grasp_times",currentGraspTimes);
        if (!result)
        {
            ROS_ERROR("the required param is not exits");
        }
        ROS_INFO("The Current Grasp Times is:%d",currentGraspTimes);

        //3) Check if the target grasp pose is ok with the: segInfo msg, currentRegistervalue, Temlist, currentGraspTimes
            //3.1 Calculate the ShiftPoseTemp:
        // bool FlagSP = getShiftPose(registerRead.response.Register_value,currentGraspTimes); 
        float TargetPoseTemp[7] = {segInfo->object_pose.pose.position.x,segInfo->object_pose.pose.position.y,segInfo->object_pose.pose.position.z,segInfo->object_pose.pose.orientation.x,segInfo->object_pose.pose.orientation.y,segInfo->object_pose.pose.orientation.z,segInfo->object_pose.pose.orientation.z};

        bool FlagSP = getShiftPose(registerRead.response.Register_value,currentGraspTimes,TargetPoseTemp,segInfo->Register_value,segInfo->GraspTimes);//Calculate and Write the Target ShiftPose to the ShiftPoseTemp
        // float ShiftPose[6]={0.0,-20.0,-5.0,0.0,0.0,0.0};//Default shift pose, It should be calculated through function getShiftPose;

            //3.2 Check if the ShiftPoseTemp is ok:
        if (!FlagSP)
        {
            ROS_ERROR("The ShiftPose Calculater stop working!!");
            /* code */
        }
        
        bool IfOperate = PoseCheck(ShiftPoseTemp);

        //Pose is OK, then calculate the shift values and call the Conveyor_tracking_controller
        if (FlagSP && IfOperate)
        {
            ROS_ERROR("Now trying to grasp target： %s, with Grasp Times when Detect:%d",segInfo->class_name,segInfo->GraspTimes);
            //The object is graspable, then it will: 1) set rosparam $(arg np)_grasp_times as currentGraspTimes+1; 2) push the gain register's to the Tem list and; 3) use the ShiftPoseTemp to initialize the ConveyorTrackingsrv and 4) then call the conveyorTrackingControl service:

            //1) set rosparam $(arg np)_grasp_times as currentGraspTimes+1:
            nh_.setParam("/left_arm_waste/left_arm_grasp_times",currentGraspTimes+1);
            ROS_ERROR("Finish setting currentGraspTimes as: %d", currentGraspTimes+1);

            //2) update the gain register's to the Tem list:
                //2.1) first to know where the value should be put:
            int listLocation = currentGraspTimes%10000;//Length of GraspFiles is 100;
                //2.2) Then to write the executed task's register's value to the listLocation of GraspFiles:
            GraspFiles[listLocation][0] = registerRead.response.Register_value;
            GraspFiles[listLocation][1] = currentGraspTimes;
            GraspFiles[listLocation][2] = SumRead;
            GraspFiles[listLocation][3] = SumValue;
            /*
            TODO
            */
           //3) use the ShiftPoseTemp to initialize the ConveyorTrackingsrv
            //3.1) call the LockLimitCalculater function to calculate the upperLimit and lowerLimit:
            bool FlagLimit = LockLimitCalculater(ShiftPoseTemp);
            if (!FlagLimit)
            {
                ROS_ERROR("Limit calculate Wrong!!! The Limt value is Wrong!!!!!!!!!!!!!!!!");             
                /* code */
            }
            int upperLimit = LimitTemp[0];
            int lowerLimit = LimitTemp[1];
            //3.2) create a ConveyorTrackingsrv and initialize it with the calculated Limits and ShiftPose:
            nachi_conveyor::nachiConveyorTracking ConveyorTrackingsrv;
            //initialize the srv.request values:
            ConveyorTrackingsrv.request.header.seq = 1;
            ConveyorTrackingsrv.request.header.stamp = ros::Time::now();
            ConveyorTrackingsrv.request.header.frame_id = "frame1";
            ConveyorTrackingsrv.request.upper_limit = upperLimit;
            ConveyorTrackingsrv.request.lower_limit = lowerLimit;
            ConveyorTrackingsrv.request.shift_pose = {ShiftPoseTemp[0],ShiftPoseTemp[1],ShiftPoseTemp[2],ShiftPoseTemp[3],ShiftPoseTemp[4],ShiftPoseTemp[5]};

            //4) call the conveyorTrackingControl
            //waiting for the serevice to comeup:
            // conveyorTrackingControlClient.call(ConveyorTrackingsrv);
            ROS_INFO("Calling the conveyor_tracking service:");
            ros::service::waitForService(conveyor_tracking);
            bool flag2 = conveyorTrackingControlClient.call(ConveyorTrackingsrv);
            if (flag2)
            {
                ROS_INFO("That's right!");
                ROS_INFO("The robot program is about to finish running!!!");
                ROS_ERROR("Finish Grasp ");
                /* code */
            }else{
                ROS_ERROR("Something goes wrong in the Robot side, Robot won't run!!!");
            }
            ROS_ERROR("Tracking control callback finish");
            /* code */
        }else{//Pose Check Wrong:
            ROS_ERROR("Inviled Pose!!! Or the ShiftPose calculate goes wrong");
        }

    }

}