#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

std::tuple<dai::Pipeline, int, int> createPipeline(bool lrcheck,
                                                   bool rectify,
                                                   bool depth_aligned,
                                                   int stereo_fps,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   int rgbScaleNumerator,
                                                   int rgbScaleDinominator) {
    dai::Pipeline pipeline;

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    xoutDepth->setStreamName("depth");
    xoutImu->setStreamName("imu");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int stereoWidth, stereoHeight, rgbWidth, rgbHeight;
    monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    stereoWidth = 1280;
    stereoHeight = 720;

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);        // Known to be best
    stereo->setRectifyEdgeFillColor(0);                              // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);  // Known to be best
    stereo->setLeftRightCheck(lrcheck);
    stereo->setSubpixel(true);
    if(depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.

    if(depth_aligned) {
        // RGB image
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        dai::node::ColorCamera::Properties::SensorResolution rgbResolution;

        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
        rgbWidth = 1920;
        rgbHeight = 1080;

        camRgb->setResolution(rgbResolution);

        rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
        rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;
        camRgb->setIspScale(rgbScaleNumerator, rgbScaleDinominator);

        camRgb->isp.link(xoutRgb->input);

        stereoWidth = rgbWidth;
        stereoHeight = rgbHeight;
    } else {
        // Stereo imges
        auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify) {
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);
        } else {
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK
    stereo->setRectifyEdgeFillColor(0);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);
    imu->out.link(xoutImu->input);
    std::cout << stereoWidth << " " << stereoHeight << " " << rgbWidth << " " << rgbHeight << std::endl;
    return std::make_tuple(pipeline, stereoWidth, stereoHeight);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_driver_node");

    std::string tfPrefix;
    int stereo_fps, confidence, LRchecktresh, imuModeParam;
    int rgbScaleNumerator, rgbScaleDinominator;
    bool lrcheck, rectify, depth_aligned;
    bool enableDotProjector, enableFloodLight;
    
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectorIntensity, floodLightIntensity;
    bool enableRosBaseTimeUpdate;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("rectify", false);

    node->declare_parameter("depth_aligned", true);
    node->declare_parameter("stereo_fps", 15);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);

    node->declare_parameter("rgbScaleNumerator", 2);
    node->declare_parameter("rgbScaleDinominator", 3);

    node->declare_parameter("angularVelCovariance", 0.02);
    node->declare_parameter("linearAccelCovariance", 0.0);

    node->declare_parameter("enableDotProjector", false);
    node->declare_parameter("enableFloodLight", false);
    node->declare_parameter("dotProjectorIntensity", 0.5);
    node->declare_parameter("floodLightIntensity", 0.5);

    node->declare_parameter("enableRosBaseTimeUpdate", false);

    // updating parameters if defined in launch file.
    node->get_parameter("tf_prefix", tfPrefix);

    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("rectify", rectify);

    node->get_parameter("depth_aligned", depth_aligned);
    node->get_parameter("stereo_fps", stereo_fps);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);

    node->get_parameter("rgbScaleNumerator", rgbScaleNumerator);
    node->get_parameter("rgbScaleDinominator", rgbScaleDinominator);

    node->get_parameter("angularVelCovariance", angularVelCovariance);
    node->get_parameter("linearAccelCovariance", linearAccelCovariance);

    node->get_parameter("enableDotProjector", enableDotProjector);
    node->get_parameter("enableFloodLight", enableFloodLight);
    node->get_parameter("dotProjectorIntensity", dotProjectorIntensity);
    node->get_parameter("floodLightIntensity", floodLightIntensity);

    node->get_parameter("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(1);

    dai::Pipeline pipeline;
    int width, height;
    bool isDeviceFound = false;
    std::tie(pipeline, width, height) = createPipeline(lrcheck,
                                                       rectify,
                                                       depth_aligned,
                                                       stereo_fps,
                                                       confidence,
                                                       LRchecktresh,
                                                       rgbScaleNumerator,
                                                       rgbScaleDinominator);

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    bool usb2Mode = false;
    bool poeMode=false;
    std::string mxId="x";

    std::cout << "Listing available devices..." << std::endl;
    for(auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                if(poeMode) {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                } else {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" is already booted on different process.  \"");
            }
        } else if(mxId == "x") {
            isDeviceFound = true;
            std::cout << "Device Found \n"; 
            device = std::make_shared<dai::Device>(pipeline);
        }
    }
    if(!isDeviceFound) {
        throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    if(!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    // Apply camera controls
    auto controlQueue = device->getInputQueue("control");

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    stereoQueue = device->getOutputQueue("depth", 30, false);
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE" && depth_aligned == false) {
        width = 640;
        height = 480;
    }
    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorIntensity(dotProjectorIntensity);
        }

        if(enableFloodLight) {
            device->setIrFloodLightIntensity(floodLightIntensity);
        }
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        converter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        rightconverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if(enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("imu/oak"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();
    
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    if(enableRosBaseTimeUpdate) {
        rgbConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    auto depthCameraInfo =
        depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height) : rightCameraInfo;
    auto depthconverter = depth_aligned ? rgbConverter : rightconverter;

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &depthconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();

    if(depth_aligned) {
        auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
        auto imgQueue = device->getOutputQueue("rgb", 30, false);
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
            imgQueue,
            node,
            std::string("color/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
        rgbPublish.addPublisherCallback();

        rclcpp::spin(node);
    } else {
        auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
        auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

        auto leftQueue = device->getOutputQueue("left", 10, false);
        auto rightQueue = device->getOutputQueue("right", 10, false);
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
            leftQueue,
            node,
            leftPubName,
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
            30,
            leftCameraInfo,
            "left");
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
            rightQueue,
            node,
            rightPubName,
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rightCameraInfo,
            "right");
        rightPublish.addPublisherCallback();
        leftPublish.addPublisherCallback();
        rclcpp::spin(node);
    }

    return 0;
}
