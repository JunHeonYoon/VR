#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/Int32.hpp>
#include <std_msgs/msg/String.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <mutex>
#include <thread>
#include <vector>
#include <Eigen/Dense>
// #include <tocabi_msgs/msg/matrix_3_4.hpp>
#include <openvr.h>
#include <fstream>
#include <iostream>

#ifndef hmdHandler_h
#define hmdHandler_h

#define M_PI 3.14159265537

#define HAPTIC_ARM_MODE 0
#define TRACKER_MODE 1
#define ULTIMATE_UPPERBODY_MODE 2
#define ULTIMATE_ARM_MODE 3
#define ULTIMATE_CONTROLLER_MODE 4
#define CONTROLLER_ONLY_MODE 5

// Initiate VR system as a VRApplication_Scene mode
// VRApplication_Scene mode : A 3D Application that will draw an environment.
// IVRSystem : provides primary data such as display configuration data, tracking data, distortion state, controller states, device properties

// IVRSystem uses "tracked Device Index" to identify a "specified device" that attached to current computer.
// typedef uint32_t TrackedDeviceIndex_t
// static const uint32_t k_unTrackedDeviceIndexInvalid = 0xFFFFFFFF
// static const uint32_t k_unMaxTrackedDeviceCount = 64 (Max device kinds..)
// static const uint32_t k_unTrackedDeviceIndex_Hmd = 0  (HMD Headset Device Index)

// TrackedDeviceClass_Invalid - There is no device at this index
// TrackedDeviceClass_HMD - The device at this index is an HMD
// TrackedDeviceClass_Controller - The device is a controller

// Functions, Knowledge to get Absolute Pose of HMd Device, Controller

// eOrigin - Tracking universe that returned poses should be relative to (one of this : TrackingUniverseSeated, TrackingUniverseStanding, TrackingUniverseRaw)
// float fPredictedSecondsToPhotonsFromNow - Number of seconds from now to predict poses for.
// set fPredictedSecondsToPhotonsFromNow to "0"  if you want to know INSTANT pose of HMD or Controller

using namespace Eigen;

typedef Matrix<float, 4, 4> Mat;
typedef float(*_FLOAT)[4];

using std::placeholders::_1;

class HMD {

public:
    // Constructor
    HMD(int arc, char* arv[]);

    // Destructor
    ~HMD();

    // Initializer
    void init();

    /* HMD, Controllers members */
public:
    void rosPublish();
    void set_ros_node(const rclcpp::Node::SharedPtr& node)
    {
        node_ = node;
    }

public:
    rclcpp::Node::SharedPtr node_;
    int argc_arg;
    char** argv_arg;

	int mode;
    bool checkHMD;
    bool checkTrackers;
    bool checkControllers;
    bool allTrackersFine;
    std_msgs::msg::Bool allTrackersFineData;

	int trackerNum;
	bool use_ultimate_tracker;

	char** serialNumber;
	bool pubPose;
    
    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracker_status_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr tracker_pose_pub;
    geometry_msgs::msg::PoseArray tracker_pose_msg;

    geometry_msgs::msg::Pose hmdVizMsg;
    geometry_msgs::msg::Pose leftControllerVizMsg;
    geometry_msgs::msg::Pose rightControllerVizMsg;
	geometry_msgs::msg::Pose* trackerVizMsg;

    
	bool button_pressed[2][4]; // [controller][button]: 0-trigger, 1-grip, 2-a, 3-b
	bool lhand_grasped;
	bool rhand_grasped;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lhand_mode_pub;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rhand_mode_pub;
	std_msgs::msg::Int32 lhand_mode_msg;
	std_msgs::msg::Int32 rhand_mode_msg;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tocabi_gui_sub;
	void tocabiGuiCallback(const std_msgs::msg::String::ConstPtr &msg);
	bool calibration_mode;

    //vr is  right-handed system
    // +y is up
    // +x is to the right
    // -z is forward
    // Distance unit is  meters

    _FLOAT map2array(Mat eigen);
    Mat map2eigen(float array[][4]);
    geometry_msgs::msg::Pose map2msg(Mat array);
    Mat coordinate_z(Mat array);
	Mat rotate_z(Mat array, float angle);
    Mat coordinate_robot(Mat array);
    Mat coordinate(Mat array);
    Vector3d rot2Euler(Matrix3f Rot);
    // tocabi_msgs::msg::Matrix3_4 makeTrackingmsg(_FLOAT array);

    Mat HMD_curEig;
    bool hmd_init_bool;

    Mat LEFTCONTROLLER_curEig;
    Mat RIGHTCONTROLLER_curEig;
	Mat* TRACKER_curEig;
    Mat ultimate_to_tracker3;

    std::ofstream writeFile;
    double init_time_;
    double cur_time_;


    /* vr component members */
public:

    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    vr::TrackedDeviceIndex_t HMD_INDEX, LEFT_CONTROLLER_INDEX, RIGHT_CONTROLLER_INDEX;
	vr::TrackedDeviceIndex_t* TRACKER_INDEX;
    vr::EVRInitError eError;
    vr::IVRSystem* VRSystem;
	vr::VRControllerState_t controllerState;

    void checkConnection();
    void RunMainLoop();

    char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];

};


#endif
