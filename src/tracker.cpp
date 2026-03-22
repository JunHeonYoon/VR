#include "tracker.h"

/* HMD Constructor IMPLEMENTATION*/
HMD::HMD(int arc, char *arv[])
{
    this->argc_arg = arc;
    this->argv_arg = arv;

    mode = CONTROLLER_ONLY_MODE;
    if (arc > 1) mode = std::stoi(arv[1]);

    checkHMD = true;
	bool allTrackersFine = true;
    switch (mode)
    {
    case HAPTIC_ARM_MODE:
        std::cout << "HAPTIC_ARM_MODE" << std::endl;
        checkTrackers = true;
        checkControllers = false;
        trackerNum = 4;
        use_ultimate_tracker = false;
        break;

    case TRACKER_MODE:
        std::cout << "TRACKER_MODE" << std::endl;
        checkTrackers = true;
        checkControllers = false;
        trackerNum = 6;
        use_ultimate_tracker = false;
        break;
    
    case ULTIMATE_UPPERBODY_MODE:
        std::cout << "ULTIMATE_UPPERBODY_MODE" << std::endl;
        checkTrackers = true;
        checkControllers = false;
        trackerNum = 5;
        use_ultimate_tracker = true;
        break;
    
    case ULTIMATE_ARM_MODE:
        std::cout << "ULTIMATE_ARM_MODE" << std::endl;
        checkTrackers = true;
        checkControllers = false;
        trackerNum = 4;
        use_ultimate_tracker = true;
        break;
    
    case ULTIMATE_CONTROLLER_MODE:
        std::cout << "ULTIMATE_CONTROLLER_MODE" << std::endl;
        checkTrackers = true;
        checkControllers = true;
        trackerNum = 3;
        use_ultimate_tracker = true;
        break;
    
    case CONTROLLER_ONLY_MODE:
        std::cout << "CONTROLLER_ONLY_MODE" << std::endl;
        checkTrackers = false;
        checkControllers = true;
        trackerNum = 0;
        use_ultimate_tracker = true;
        break;
    
    default:
        std::cout << "Unidentified Mode!!!" << std::endl;
        checkTrackers = false;
        checkControllers = false;
        trackerNum = 0;
        use_ultimate_tracker = false;
        break;
    }

    serialNumber = new char*[trackerNum];
    for (int i = 0; i < trackerNum; i++)
    {
        serialNumber[i] = new char[15];
    }
    TRACKER_INDEX = new vr::TrackedDeviceIndex_t[trackerNum];
    TRACKER_curEig = new Mat[trackerNum];
    trackerVizMsg = new geometry_msgs::msg::Pose[trackerNum];

    hmd_init_bool = false;
    pubPose = true;
    ultimate_to_tracker3 << 1, 0, 0, 0,
                            0, 0, -1, 0,
                            0, 1, 0, 0,
                            0, 0, 0, 1;

    memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));

    lhand_button_msg.data.resize(4);
    lhand_button_msg.data = {0, 0, 0, 0};
    rhand_button_msg.data.resize(4);
    rhand_button_msg.data = {0, 0, 0, 0};
}

/* HMD Destructor IMPLEMENTATION*/
HMD::~HMD()
{
    // memory release is done in Shutdown
    for (int i = 0; i < trackerNum; i++)
    {
        delete[] serialNumber[i];
    }
    delete[] serialNumber;
    delete[] TRACKER_INDEX;
    delete[] TRACKER_curEig;
    delete[] trackerVizMsg;
}

std::string GetTrackedDeviceString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
    uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
    if (unRequiredBufferLen == 0)
        return "";

    char *pchBuffer = new char[unRequiredBufferLen];
    unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
    std::string sResult = pchBuffer;
    delete[] pchBuffer;
    return sResult;
}

/* HMD Initialization IMPLEMENTATION*/
void HMD::init()
{
    if (!node_) {
        throw std::runtime_error("ROS2 node is not set. Call HMD::set_ros_node() before init().");
    }

    // publishers (기존 토픽/ QoS 유지)
    tracker_status_pub = node_->create_publisher<std_msgs::msg::Bool>("TRACKERSTATUS", rclcpp::QoS(1000));
    tracker_pose_pub   = node_->create_publisher<geometry_msgs::msg::PoseArray>("tracker_pose", rclcpp::QoS(1));

    lhand_mode_pub = node_->create_publisher<std_msgs::msg::Int32>("lhand_mode", rclcpp::QoS(1));  // TODO: check topic name
    rhand_mode_pub = node_->create_publisher<std_msgs::msg::Int32>("rhand_mode", rclcpp::QoS(1));

    lhand_button_pub = node_->create_publisher<std_msgs::msg::Int32MultiArray>("lhand_button", rclcpp::QoS(1));
    rhand_button_pub = node_->create_publisher<std_msgs::msg::Int32MultiArray>("rhand_button", rclcpp::QoS(1));

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            button_pressed[i][j] = false;
        }
    }
    lhand_grasped = false;
    rhand_grasped = false;

    tocabi_gui_sub = node_->create_subscription<std_msgs::msg::String>("/tocabi/guilog", 1000, std::bind(&HMD::tocabiGuiCallback, this, _1));
    calibration_mode = false;

    // (필요한 viz pub들도 여기서 node_로 생성)
    // hmd_viz_pub = node_->create_publisher<geometry_msgs::msg::Pose>("hmd_viz", rclcpp::QoS(1));
    // for (int i=0; i<trackerNum; ++i) tracker_viz_pub[i] = node_->create_publisher<geometry_msgs::msg::Pose>(...);

    // 시간 초기화도 node_ 기준
    init_time_ = node_->now().seconds();

    if(mode == HAPTIC_ARM_MODE)
        tracker_pose_msg.poses.resize(5);
    else if (mode == CONTROLLER_ONLY_MODE)
        tracker_pose_msg.poses.resize(3);
    else
        tracker_pose_msg.poses.resize(7);

    eError = vr::VRInitError_None;
    VRSystem = vr::VR_Init(&eError, vr::VRApplication_Background);

    if (eError != vr::VRInitError_None)
    {
        VRSystem = nullptr;
        throw std::runtime_error(std::string("VR_Init failed: ") + vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    }

    std::cout << "Start Connection Check" << std::endl;
    checkConnection();

    hmd_init_bool = true;

    if (writeFile.is_open())
        writeFile.close();
    writeFile.open("tracker_log.txt", std::ofstream::out | std::ofstream::app);
    writeFile << std::fixed << std::setprecision(8);
}


void HMD::RunMainLoop()
{
    rclcpp::WallRate r(130); // ROS2: WallRate 사용
    while (rclcpp::ok())
    {
        rosPublish();
        r.sleep();
    }
}


/* Check HMD, Controllers connection state */
void HMD::checkConnection()
{
    std::cout << "Maximum Number of Device that can be tracked: " << vr::k_unMaxTrackedDeviceCount << std::endl;
    std::cout << "Number of trackers to find: " << trackerNum << std::endl;
    std::cout << "Please Connect Your HMD and Six Trackers to Start This Program" << std::endl;
    VRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount);
    while (true)
    {
        int HMD_count = 0;
        int controller_count = 0;
        int tracker_count = 0;

        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
        {
            vr::ETrackedDeviceClass trackedStatus = VRSystem->GetTrackedDeviceClass(i);

            switch (int(trackedStatus))
            {
            case 0:
                continue;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
            {
                this->HMD_INDEX = i;
                HMD_count += 1;
                continue;
            }
            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
            {
                std::cout << "controller:   " << i << std::endl;
                vr::ETrackedControllerRole controllerRole = VRSystem->GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t(i));
                if (controllerRole == 1)
                {
                    std::cout << "Left controller identified! Left controller idx: " << i << std::endl;
                    this->LEFT_CONTROLLER_INDEX = i;
                }
                else if (controllerRole == 2)
                {
                    std::cout << "Right controller identified! Right controller idx: " << i << std::endl;
                    this->RIGHT_CONTROLLER_INDEX = i;
                }
                controller_count += 1;
                continue;
            }
            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
            {

                // if (m_rTrackedDevicePose[i].bPoseIsValid)
                // {
                std::cout << "Tracker " << tracker_count << " identified! Idx is " << i << std::endl;
                vr::VRSystem()->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, serialNumber[tracker_count], 15*sizeof(char));
                printf("Serial Number = %s \n", serialNumber[tracker_count]);
                this->TRACKER_INDEX[tracker_count] = i;
                tracker_count += 1;
                // }
                continue;
            }
            case 4:
                continue;
            case 5:
                continue;
            }
        }
        
        // Only use HMD
        if (!checkControllers && !checkTrackers && HMD_count == 1)
        {
            break;
        }
        // Use HMD+controllers
        if (checkControllers && !checkTrackers && (HMD_count == 1 && controller_count == 2))
        {
            break;
        }
        // Use trackers
        if (!checkHMD && !checkControllers && checkTrackers && (tracker_count == trackerNum))
        {
            break;
        }
        // Use HMD+trackers
        if (!checkControllers && checkTrackers && (HMD_count == 1 && tracker_count >= trackerNum))
        {
            break;
        }
        // Use HMD++controllers+trackers
        if (checkControllers && checkTrackers && (HMD_count == 1 && controller_count == 2 && tracker_count == trackerNum))
        {
            break;
        }
    }

    if (!checkControllers && !checkTrackers)
    {
        std::cout << "HMD is identified..." << std::endl;
        std::cout << "All Specified Connection Identified.. Start VR system.." << std::endl;
    }
    else if (checkControllers && !checkTrackers)
    {
        std::cout << "One HMD and Two controllers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }
    else if (!checkControllers && checkTrackers)
    {
        std::cout << "One HMD and " << trackerNum << " trackers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }
    else
    {
        std::cout << "One HMD, Two controllers and " << trackerNum << " trackers are identified..." << std::endl;
        std::cout << "Start VR system and ROS NODE" << std::endl;
    }
}

void HMD::rosPublish()
{
    VRSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount);
    // HMD : Send only rotation parameters(euler or quarternion)
    // controller : Send rotation & translation parameters(w.r.t current HMD Cordinate)
    HMD_curEig = map2eigen(m_rTrackedDevicePose[HMD_INDEX].mDeviceToAbsoluteTracking.m);
    hmdVizMsg = map2msg(coordinate_robot(HMD_curEig));
    
    allTrackersFine = true;
    if (checkControllers)
    {
        LEFTCONTROLLER_curEig = map2eigen(m_rTrackedDevicePose[LEFT_CONTROLLER_INDEX].mDeviceToAbsoluteTracking.m);
        if (use_ultimate_tracker) LEFTCONTROLLER_curEig *= ultimate_to_tracker3;
        RIGHTCONTROLLER_curEig = map2eigen(m_rTrackedDevicePose[RIGHT_CONTROLLER_INDEX].mDeviceToAbsoluteTracking.m);
        if (use_ultimate_tracker) RIGHTCONTROLLER_curEig *= ultimate_to_tracker3;
        
        leftControllerVizMsg = map2msg(rotate_z(coordinate_robot(LEFTCONTROLLER_curEig), M_PI/2));
        rightControllerVizMsg = map2msg(rotate_z(coordinate_robot(RIGHTCONTROLLER_curEig), -M_PI/2));
        allTrackersFine *= m_rTrackedDevicePose[LEFT_CONTROLLER_INDEX].bPoseIsValid;
        // std::cout << LEFT_CONTROLLER_INDEX << m_rTrackedDevicePose[LEFT_CONTROLLER_INDEX].bPoseIsValid << std::endl;
        allTrackersFine *= m_rTrackedDevicePose[RIGHT_CONTROLLER_INDEX].bPoseIsValid;
        // std::cout << RIGHT_CONTROLLER_INDEX << m_rTrackedDevicePose[RIGHT_CONTROLLER_INDEX].bPoseIsValid << std::endl;

        // Publish button states
        // TODO check mode numbers
        if (VRSystem->GetControllerState(LEFT_CONTROLLER_INDEX, &controllerState, sizeof(controllerState)))
        {
            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) // left trigger button on
            {
                lhand_button_msg.data[0] = 1;

                if(!button_pressed[0][0]) // if previous left trigger button is off 
                {
                    button_pressed[0][0] = true;
                    lhand_grasped = !lhand_grasped;
                    lhand_mode_msg.data = lhand_grasped ? 1 : 0;    // 0: Grasp 1: Stretch
                    lhand_mode_pub->publish(lhand_mode_msg);
                    std::cout << "Left Controller Trigger Pressed!" << std::endl;
                }
            }
            else
            {
                lhand_button_msg.data[0] = 0;
                button_pressed[0][0] = false;
            }

            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) // left grip button on
            {
                lhand_button_msg.data[1] = 1;

                if(!button_pressed[0][1]) // if previous left grip button is off 
                {
                    button_pressed[0][1] = true;
                    lhand_mode_msg.data = 2;    // Five
                    lhand_mode_pub->publish(lhand_mode_msg);
                    std::cout << "Left Controller Grip Pressed!" << std::endl;
                }
            }
            else
            {
                lhand_button_msg.data[1] = 0;
                button_pressed[0][1] = false;
            }

            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_A)) // left A button on
            {
                lhand_button_msg.data[2] = 1;

                if(!button_pressed[0][2]) // if previous left A button is off 
                {
                    button_pressed[0][2] = true;
                    lhand_mode_msg.data = 3;    // ThumbsUp
                    lhand_mode_pub->publish(lhand_mode_msg);
                    std::cout << "Left Controller X Button Pressed!" << std::endl;
                }
            }
            else
            {
                lhand_button_msg.data[2] = 0;
                button_pressed[0][2] = false;
            }
            
            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_IndexController_B)) // left B button on
            {
                lhand_button_msg.data[3] = 1;

                if(!button_pressed[0][3]) // if previous left B button is off 
                {
                    button_pressed[0][3] = true;
                    lhand_mode_msg.data = 4;    // V
                    lhand_mode_pub->publish(lhand_mode_msg);
                    std::cout << "Left Controller Y Button Pressed!" << std::endl;
                }
            }
            else
            {
                lhand_button_msg.data[3] = 0;
                button_pressed[0][3] = false;
            }
            // if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Axis0))
            // {
            //     std::cout << "Left Controller Joystick Pressed!" << std::endl;
            // }
            // std::cout << "Left Controller Joystick Value: " << controllerState.rAxis[0].x << ", " << controllerState.rAxis[0].y << std::endl;
            // std::cout << "Left Controller Trigger Value: " << controllerState.rAxis[1].x << std::endl;
            // std::cout << "Left Controller Grip Value: " << controllerState.rAxis[2].x << std::endl;
        }
        if (VRSystem->GetControllerState(RIGHT_CONTROLLER_INDEX, &controllerState, sizeof(controllerState)))
        {
            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger))
            {
                rhand_button_msg.data[0] = 1;
                if(!button_pressed[1][0])
                {
                    button_pressed[1][0] = true;
                    rhand_grasped = !rhand_grasped;
                    rhand_mode_msg.data = rhand_grasped ? 1 : 0;    // 0: Grasp 1: Stretch
                    rhand_mode_pub->publish(rhand_mode_msg);
                    std::cout << "Right Controller Trigger Pressed!" << std::endl;
                }
            }
            else
            {
                rhand_button_msg.data[0] = 0;
                button_pressed[1][0] = false;
            }

            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip))
            {
                rhand_button_msg.data[1] = 1;
                if(!button_pressed[1][1])
                {
                    button_pressed[1][1] = true;
                    rhand_mode_msg.data = 2;    // Five
                    rhand_mode_pub->publish(rhand_mode_msg);
                    std::cout << "Right Controller Grip Pressed!" << std::endl;
                }
            }
            else
            {
                rhand_button_msg.data[1] = 0;
                button_pressed[1][1] = false;
            }
            
            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_A))
            {
                rhand_button_msg.data[2] = 1;
                if(!button_pressed[1][2])
                {
                    button_pressed[1][2] = true;
                    rhand_mode_msg.data = 3;    // ThumbsUp
                    rhand_mode_pub->publish(rhand_mode_msg);
                    std::cout << "Right Controller A Button Pressed!" << std::endl;
                }
            }
            else
            {
                rhand_button_msg.data[2] = 0;
                button_pressed[1][2] = false;
            }

            if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_IndexController_B))
            {
                rhand_button_msg.data[3] = 1;
                if(!button_pressed[1][3])
                {
                    button_pressed[1][3] = true;
                    rhand_mode_msg.data = 4;    // V
                    rhand_mode_pub->publish(rhand_mode_msg);
                    std::cout << "Right Controller B Button Pressed!" << std::endl;
                }
            }
            else
            {
                rhand_button_msg.data[3] = 0;
                button_pressed[1][3] = false;
            }
            // if (controllerState.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Axis0))
            // {
            //     std::cout << "Right Controller Joystick Pressed!" << std::endl;
            // }
            // std::cout << "Right Controller Joystick Value: " << controllerState.rAxis[0].x << ", " << controllerState.rAxis[0].y << std::endl;
            // std::cout << "Right Controller Trigger Value: " << controllerState.rAxis[1].x << std::endl;
            // std::cout << "Right Controller Grip Value: " << controllerState.rAxis[2].x << std::endl;

            lhand_button_pub->publish(lhand_button_msg);
            rhand_button_pub->publish(rhand_button_msg);

        }
    }
    if (checkTrackers)
    {
        for (int i = 0; i < trackerNum; i++)
        {
            TRACKER_curEig[i] = map2eigen(m_rTrackedDevicePose[TRACKER_INDEX[i]].mDeviceToAbsoluteTracking.m);
            if (use_ultimate_tracker) TRACKER_curEig[i] *= ultimate_to_tracker3;
            trackerVizMsg[i] = map2msg(coordinate_robot(TRACKER_curEig[i]));
            allTrackersFine *= m_rTrackedDevicePose[TRACKER_INDEX[i]].bPoseIsValid;
            // std::cout << TRACKER_INDEX[i] << m_rTrackedDevicePose[TRACKER_INDEX[i]].bPoseIsValid << std::endl;
        }
    }

    if (pubPose)
    {
        allTrackersFineData.data = allTrackersFine;
        tracker_status_pub->publish(allTrackersFineData);
        // std::cout << "allTrackersFine: " << allTrackersFine << std::endl;

        if (allTrackersFine)
        {
            tracker_pose_msg.header.frame_id="tracker_base";
            tracker_pose_msg.header.stamp = node_->now();
            switch (mode)
            {
            case HAPTIC_ARM_MODE:
                for (int i = 0; i < trackerNum; i++)
                {
                    if (std::string(serialNumber[i]) == "LHR-B979AA9E" || std::string(serialNumber[i]) == "LHR-5567029A" || std::string(serialNumber[i]) == "LHR-7F3336E9") // waist(0)
                    {
                        tracker_pose_msg.poses[0] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-3F2A7A7B" || std::string(serialNumber[i]) == "LHR-D74F7D1A" || std::string(serialNumber[i]) == "LHR-B90B28C6") // chest(1)
                    {
                        tracker_pose_msg.poses[1] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-7330E069" || std::string(serialNumber[i]) == "LHR-78CF9EE8" || std::string(serialNumber[i]) == "LHR-5CC57528") // left shoulder(2)
                    {
                        tracker_pose_msg.poses[2] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-3C32FE4B" || std::string(serialNumber[i]) == "LHR-172B3493" || std::string(serialNumber[i]) == "LHR-2ADDDA7C") // right shoulder(3)
                    {
                        tracker_pose_msg.poses[3] = trackerVizMsg[i];
                    }
                }                
                tracker_pose_msg.poses[4] = hmdVizMsg;  // head
                break;

            case TRACKER_MODE:
                for (int i = 0; i < trackerNum; i++)
                {
                    if (std::string(serialNumber[i]) == "LHR-B979AA9E" || std::string(serialNumber[i]) == "LHR-5567029A" || std::string(serialNumber[i]) == "LHR-7F3336E9") // waist(0)
                    {
                        tracker_pose_msg.poses[0] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-3F2A7A7B" || std::string(serialNumber[i]) == "LHR-D74F7D1A" || std::string(serialNumber[i]) == "LHR-B90B28C6") // chest(1)
                    {
                        tracker_pose_msg.poses[1] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-7330E069" || std::string(serialNumber[i]) == "LHR-78CF9EE8" || std::string(serialNumber[i]) == "LHR-5CC57528") // left shoulder(2)
                    {
                        tracker_pose_msg.poses[2] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-8C0A4142" || std::string(serialNumber[i]) == "LHR-CA171B68" || std::string(serialNumber[i]) == "LHR-E54DE63D") // left hand(3)
                    {
                        tracker_pose_msg.poses[3] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-3C32FE4B" || std::string(serialNumber[i]) == "LHR-172B3493" || std::string(serialNumber[i]) == "LHR-2ADDDA7C") // right shoulder(4)
                    {
                        tracker_pose_msg.poses[4] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "LHR-5423DE85" || std::string(serialNumber[i]) == "LHR-88A2CD57" || std::string(serialNumber[i]) == "LHR-3BE6ECE8") // right hand(5)
                    {
                        tracker_pose_msg.poses[5] = trackerVizMsg[i];
                    }
                }
                tracker_pose_msg.poses[6] = hmdVizMsg;  // head
                break;

            case ULTIMATE_UPPERBODY_MODE:
                // waist: fixed
                tracker_pose_msg.poses[0].position.x = -1.0;
                tracker_pose_msg.poses[0].position.z = -1.0;
                tracker_pose_msg.poses[0].orientation.y = 1.0;
                for (int i = 0; i < trackerNum; i++)
                {
                    if (std::string(serialNumber[i]) == "41-A33P01155") // chest
                    {
                        tracker_pose_msg.poses[1] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P00502") // left shoulder
                    {
                        tracker_pose_msg.poses[2] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P00501") // left hand
                    {
                        tracker_pose_msg.poses[3] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P02176") // right shoulder
                    {
                        tracker_pose_msg.poses[4] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P02298") // right hand
                    {
                        tracker_pose_msg.poses[5] = trackerVizMsg[i];
                    }
                }
                tracker_pose_msg.poses[6] = hmdVizMsg;  // head
                break;
                
            case ULTIMATE_ARM_MODE:
                // waist: fixed
                tracker_pose_msg.poses[0].position.x = -1.0;
                tracker_pose_msg.poses[0].position.z = -1.0;
                tracker_pose_msg.poses[0].orientation.y = 1.0;
                // chest: fixed
                tracker_pose_msg.poses[1].position.x = -0.8;
                tracker_pose_msg.poses[1].position.z = -0.8;
                tracker_pose_msg.poses[1].orientation.y = 1.0;
                for (int i = 0; i < trackerNum; i++)
                {
                    if (std::string(serialNumber[i]) == "41-A33P00502") // left shoulder
                    {
                        tracker_pose_msg.poses[2] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P00501") // left hand
                    {
                        tracker_pose_msg.poses[3] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P02176") // right shoulder
                    {
                        tracker_pose_msg.poses[4] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P02298") // right hand
                    {
                        tracker_pose_msg.poses[5] = trackerVizMsg[i];
                    }
                }
                // head: fixed
                if (calibration_mode) {
                    tracker_pose_msg.poses[6].orientation.x = 0.0;
                    tracker_pose_msg.poses[6].orientation.y = 0.0;
                    tracker_pose_msg.poses[6].orientation.z = 0.0;
                    tracker_pose_msg.poses[6].orientation.w = 1.0;
                }
                else{
                    tracker_pose_msg.poses[6].orientation.x = 0.0;
                    tracker_pose_msg.poses[6].orientation.y = 0.1494381;
                    tracker_pose_msg.poses[6].orientation.z = 0.0;
                    tracker_pose_msg.poses[6].orientation.w = 0.9887711;
                }
                break;
                
            case ULTIMATE_CONTROLLER_MODE:
                tracker_pose_msg.poses[0].position.x = -1.0;
                tracker_pose_msg.poses[0].position.z = -1.0;
                tracker_pose_msg.poses[0].orientation.y = 1.0;  // waist: fixed
                for (int i = 0; i < trackerNum; i++)
                {
                    if (std::string(serialNumber[i]) == "41-A33P01155") // chest
                    {
                        tracker_pose_msg.poses[1] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P00502") // left shoulder
                    {
                        tracker_pose_msg.poses[2] = trackerVizMsg[i];
                    }
                    else if (std::string(serialNumber[i]) == "41-A33P02176") // right shoulder
                    {
                        tracker_pose_msg.poses[4] = trackerVizMsg[i];
                    }
                }
                tracker_pose_msg.poses[3] = leftControllerVizMsg;   // left hand
                tracker_pose_msg.poses[5] = rightControllerVizMsg;  // right hand
                tracker_pose_msg.poses[6] = hmdVizMsg;  // head
                break;

            case CONTROLLER_ONLY_MODE:
                tracker_pose_msg.poses[0] = leftControllerVizMsg;   // left hand
                tracker_pose_msg.poses[1] = rightControllerVizMsg;  // right hand
                tracker_pose_msg.poses[2] = hmdVizMsg;  // head
                break;
            }

            tracker_pose_pub->publish(tracker_pose_msg);
        }

        cur_time_ = node_->now().seconds();
        // writeFile << cur_time_ - init_time_ << "\t";
        // for (int i=0; i<trackerNum; i++)
        // {
        //     writeFile << trackerVizMsg[i].position.x << "\t" << trackerVizMsg[i].position.y << "\t" << trackerVizMsg[i].position.z << "\t";
        //     writeFile << trackerVizMsg[i].orientation.x << "\t" << trackerVizMsg[i].orientation.y << "\t" << trackerVizMsg[i].orientation.z << "\t" << trackerVizMsg[i].orientation.w << "\t";
        // }
        // writeFile << allTrackersFine << std::endl;
    }
}


void HMD::tocabiGuiCallback(const std_msgs::msg::String::ConstPtr &msg)
{
    if (msg->data == "RESET POSE CALIBRATION"){
        calibration_mode = true;
        std::cout << "Calibration Start!" << std::endl;
    }
    else if (msg->data.substr(0, 15) == "Left Arm Length"){
        calibration_mode = false;
        std::cout << "Calibration Done!" << std::endl;
        std::cout << msg->data << std::endl;
    }
}

/* Private members IMPLEMENTATIOn*/
_FLOAT HMD::map2array(Mat eigen)
{
    // Return 4x4 Eigen matrix to 3x4 c++ matrix
    _FLOAT array = new float[3][4];

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            array[row][col] = eigen(row, col);
        }
    }
    return array;
}

Mat HMD::map2eigen(float array[][4])
{
    // Return 3x4 c++ matrix to 4x4 Eigen matrix
    Mat eigen;
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            eigen(row, col) = array[row][col];
        }
    }
    eigen(3, 0) = 0;
    eigen(3, 1) = 0;
    eigen(3, 2) = 0;
    eigen(3, 3) = 1;

    return eigen;
}

geometry_msgs::msg::Pose HMD::map2msg(Mat array)
{
    geometry_msgs::msg::Pose trackerMsg;
    trackerMsg.position.x = array(0, 3);
    trackerMsg.position.y = array(1, 3);
    trackerMsg.position.z = array(2, 3);

    Matrix3f mat = array.block(0, 0, 3, 3);
    Quaternionf q(mat);
    trackerMsg.orientation.x = q.x();
    trackerMsg.orientation.y = q.y();
    trackerMsg.orientation.z = q.z();
    trackerMsg.orientation.w = q.w();

    return trackerMsg;
}

Mat HMD::coordinate_z(Mat array)
{

    Mat y_r;
    Mat x_r;
    Mat z_r;

    y_r << 0, 0, -1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    x_r << 1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    z_r << -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Mat x_r_array = x_r * y_r * array;

    x_r_array.col(0) = x_r_array.col(0) * -1;
    x_r_array.col(1) = x_r_array.col(1) * -1;

    return x_r_array;
}

Mat HMD::rotate_z(Mat array, float angle)
{
    Mat z_r;
    z_r << cos(angle), -sin(angle), 0, 0,
           sin(angle), cos(angle), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;

    return array * z_r;
}

Mat HMD::coordinate_robot(Mat array)
{

    Mat local_coordinate_rotation;

    local_coordinate_rotation << 0, -1, 0, 0,
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1;
    Mat vr_to_robot;
    vr_to_robot << 0, 0, -1, 0,
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    return vr_to_robot * array * local_coordinate_rotation;
    // return array;
}

Mat HMD::coordinate(Mat array)
{
    Mat y_r;
    Mat x_r;
    Mat z_r;

    y_r << 0, 0, -1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    x_r << 1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;

    return x_r * y_r * array;
}

// tocabi_msgs::matrix_3_4 HMD::makeTrackingmsg(_FLOAT array)
// {
//     tocabi_msgs::matrix_3_4 msg;

//     for (int i = 0; i < 4; i++)
//         msg.firstRow.push_back(array[0][i]);
//     for (int i = 0; i < 4; i++)
//         msg.secondRow.push_back(array[1][i]);
//     for (int i = 0; i < 4; i++)
//         msg.thirdRow.push_back(array[2][i]);

//     return msg;
// }

Vector3d HMD::rot2Euler(Matrix3f Rot)
{
    double beta;
    Eigen::Vector3d angle;
    beta = -asin(Rot(2, 0));
    double DEG2RAD = 3.14 / 180;
    if (abs(beta) < 90 * DEG2RAD)
        beta = beta;
    else
        beta = 180 * DEG2RAD - beta;

    angle(0) = atan2(Rot(2, 1), Rot(2, 2) + 1E-37); // roll
    angle(2) = atan2(Rot(1, 0), Rot(0, 0) + 1E-37); // pitch
    angle(1) = beta;                                // yaw

    return angle;
}