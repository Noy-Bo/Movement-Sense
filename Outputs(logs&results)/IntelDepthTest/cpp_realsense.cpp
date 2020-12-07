#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <cubemos/engine.h>
#include <cubemos/skeleton_tracking.h>

#include "../samples.h"

using CUBEMOS_SKEL_Buffer_Ptr = std::unique_ptr<CM_SKEL_Buffer, void (*)(CM_SKEL_Buffer*)>;
using CUBEMOS_SKEL_TrackingPipeline_Ptr = std::unique_ptr<CM_SKEL_TrackingContext, void (*)(CM_SKEL_TrackingContext*)>;

static const cv::Scalar skeletonColor = cv::Scalar(100, 254, 213);
static const cv::Scalar jointColor = cv::Scalar(222, 55, 22);

struct cmPoint {
    float color_pixel[2];
    float point3d[3];
    std::string to_string() const
    {
        char buffer[100];
        int cx = snprintf(buffer, 100, "(%.2f, %.2f, %.2f)", point3d[0], point3d[1], point3d[2]);
        return std::string(buffer);
    }
};

cmPoint
get_skeleton_point_3d(rs2::depth_frame const& depthFrame, int x, int y)
{
    // Get the distance at the given pixel
    auto distance = depthFrame.get_distance(x, y);

    cmPoint point;
    point.color_pixel[0] = static_cast<float>(x);
    point.color_pixel[1] = static_cast<float>(y);

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = depthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
	intr.width = 640;
    rs2_deproject_pixel_to_point(point.point3d, &intr, point.color_pixel, distance);
	
	std::cout << "x: " << x << "y: " << y << "XYZ: " << point.point3d;

    return point;
}

CUBEMOS_SKEL_Buffer_Ptr
create_skel_buffer()
{
    return CUBEMOS_SKEL_Buffer_Ptr(new CM_SKEL_Buffer(), [](CM_SKEL_Buffer* pb) {
        cm_skel_release_buffer(pb);
        delete pb;
    });
}

CUBEMOS_SKEL_TrackingPipeline_Ptr
create_tracking_pipeline()
{
    CM_SKEL_TrackingContext* ptr = nullptr;
    auto retcode =
      cm_skel_create_tracking_context_options(&ptr, CM_SKEL_TrackingSimilarityMetric::CM_IOU, 25, CM_TRACKING_FULLBODY_EDGE);
    return CUBEMOS_SKEL_TrackingPipeline_Ptr(
      ptr, [](CM_SKEL_TrackingContext* ptr) { cm_skel_release_tracking_context(&ptr); });
}

/*
Render skeletons and tracking ids on top of the color image
*/
inline void
renderSkeletons(const CM_SKEL_Buffer* skeletons_buffer, rs2::depth_frame const& depth_frame, cv::Mat& image)
{
    CV_Assert(image.type() == CV_8UC3);
    const cv::Point2f absentKeypoint(-1.0f, -1.0f);

    const std::vector<std::pair<int, int>> limbKeypointsIds = { { 1, 2 },   { 1, 5 },   { 2, 3 }, { 3, 4 },  { 5, 6 },
                                                                { 6, 7 },   { 1, 8 },   { 8, 9 }, { 9, 10 }, { 1, 11 },
                                                                { 11, 12 }, { 12, 13 }, { 1, 0 }, { 0, 14 }, { 14, 16 },
                                                                { 0, 15 },  { 15, 17 } };

    for (int i = 0; i < skeletons_buffer->numSkeletons; i++) {
        CV_Assert(skeletons_buffer->skeletons[i].numKeyPoints == 18);

        int id = skeletons_buffer->skeletons[i].id;
        cv::Point2f keyPointHead(skeletons_buffer->skeletons[i].keypoints_coord_x[0],
                                 skeletons_buffer->skeletons[i].keypoints_coord_y[0]);

        for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
            const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
                                       skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
            if (keyPoint != absentKeypoint) {
                cv::circle(image, keyPoint, 4, jointColor, -1);

                // get the 3d point and render it on the joints
                cmPoint point3d =
                  get_skeleton_point_3d(depth_frame, static_cast<int>(keyPoint.x), static_cast<int>(keyPoint.y));
                cv::putText(image, point3d.to_string(), keyPoint, cv::FONT_HERSHEY_SIMPLEX, 0.25, jointColor);
				//std::cout << "x: " + point3d.;
            }
        }

        for (const auto& limbKeypointsId : limbKeypointsIds) {
            const cv::Point2f keyPointFirst(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.first],
                                            skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.first]);

            const cv::Point2f keyPointSecond(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.second],
                                             skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.second]);

            if (keyPointFirst == absentKeypoint || keyPointSecond == absentKeypoint) {
                continue;
            }

            cv::line(image, keyPointFirst, keyPointSecond, skeletonColor, 2, cv::LINE_AA);
        }
        for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
            const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
                                       skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
            if (keyPoint != absentKeypoint) {
                // found a valid keypoint and displaying the skeleton tracking id next to it
                cv::putText(image,
                            (std::to_string(id)),
                            cv::Point2f(keyPoint.x, keyPoint.y - 20),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            skeletonColor);
                break;
            }
        }
    }
}

int
main(int argc, char* argv[])
{
	
    // set up the intel realsense pipeline
    rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
	rs2::device device;
	rs2::pipeline_profile profile;
	rs2::sensor sensor;


	cfg.enable_device_from_file("C:/Age_Estimation_Project/bag_files/dana_depth_test.bag",false);
	pipe.start(cfg);
	
	
	//pipe.start(cfg);
	//device = pipe.get_active_profile().get_device();
	
   /* if (ctx.query_devices().size() == 0) {
        EXIT_PROGRAM("No realsense device connected.");
    }*/

    cfg.enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_BGR8, 30);
    rs2::align align_to_color(RS2_STREAM_DEPTH);

    //rs2::pipeline_profile profile;
   

    //// set high density mode with full laser power
    //auto sensor = profile.get_device().first<rs2::depth_sensor>();
    //sensor.set_option(RS2_OPTION_LASER_POWER, 100.0);

    //auto device_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
    //std::cout << "Intel Realsense " << device_name << " " << sensor.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
    //          << " is connected." << std::endl;

    //pipe.stop();
    //if (strcmp(device_name, "L500 Depth Sensor") == 0) {
    //    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 1024, 768, RS2_FORMAT_Z16, 30);
    //}
    //else {
    //    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, RS2_FORMAT_Z16, 30);
    //}
	

    //auto range = sensor.get_option_range(RS2_OPTION_VISUAL_PRESET);
   /* for (auto i = range.min; i < range.max; i += range.step)
        if (std::string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "High Density")
            sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);*/

    CM_TargetComputeDevice enInferenceMode = CM_TargetComputeDevice::CM_CPU;

    if (argc > 1) {
        if (strcmp(argv[1], "CPU") == 0 || strcmp(argv[1], "MYRIAD") == 0 || strcmp(argv[1], "GPU") == 0) {
            if (strcmp(argv[1], "MYRIAD") == 0)
                enInferenceMode = CM_TargetComputeDevice::CM_MYRIAD;
            else if (strcmp(argv[1], "CPU") == 0)
                enInferenceMode = CM_TargetComputeDevice::CM_CPU;
            else if (strcmp(argv[1], "GPU") == 0)
                enInferenceMode = CM_TargetComputeDevice::CM_GPU;
        }
    }

    // set up the cubemos skeleton tracking api pipeline
    CM_SKEL_Handle* handle = nullptr;
    // Output all messages with severity level INFO or higher to the console and to a file
    cm_initialise_logging(CM_LogLevel::CM_LL_INFO, true, default_log_dir().c_str());

    CM_ReturnCode retCode = cm_skel_create_handle(&handle, default_license_dir().c_str());
    CHECK_HANDLE_CREATION(retCode);

    std::string modelName = default_model_dir();
    if (enInferenceMode == CM_TargetComputeDevice::CM_CPU) {
        modelName += std::string("/fp32/skeleton-tracking.cubemos");
    }
    else {
        modelName += std::string("/fp16/skeleton-tracking.cubemos");
    }
    retCode = cm_skel_load_model(handle, enInferenceMode, modelName.c_str());
    if (retCode != CM_SUCCESS) {
        EXIT_PROGRAM("Model loading failed.");
    }

    std::string cvWindowName = "cubemos: skeleton tracking with intel realsense camera C/C++";
    cv::namedWindow(cvWindowName, cv::WINDOW_NORMAL);
    cv::setWindowProperty(cvWindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cv::Mat capturedFrame;

    const int nHeight = 256; // height of the image with which the DNN model will run inference

    // continue to loop through acquisition and display until the escape key is hit
    int frameCount = 0;
    std::string fpsTest = "Frame rate: ";

    // start measuring the time taken for execution
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();

    auto tracking_pipeline = create_tracking_pipeline();

    while (cv::waitKey(1) != 27) {
        // capture image
        rs2::frameset data = pipe.wait_for_frames();
        data = align_to_color.process(data);


        rs2::frame colorFrame = data.get_color_frame();
        rs2::frame depthFrame = data.get_depth_frame();

        capturedFrame = cv::Mat(
          cv::Size(colorFrame.as<rs2::video_frame>().get_width(), colorFrame.as<rs2::video_frame>().get_height()),
          CV_8UC3,
          (void*)colorFrame.get_data(),
          cv::Mat::AUTO_STEP);

        // exit the loop if the captured frame is empty
        if (capturedFrame.empty()) {
            std::cerr << "No new frame could be captured using the input source. Exiting the loop." << std::endl;
            break;
        }

        CM_Image imagePresent = {
            capturedFrame.data,         CM_UINT8, capturedFrame.cols, capturedFrame.rows, capturedFrame.channels(),
            (int)capturedFrame.step[0], CM_HWC
        };

        CUBEMOS_SKEL_Buffer_Ptr skeletons = create_skel_buffer();
        // Run Skeleton Tracking and display the results
        retCode = cm_skel_estimate_keypoints(handle, &imagePresent, nHeight, skeletons.get());

        // track the skeletons in case of successful skeleton estimation
        if (retCode == CM_SUCCESS) {
            // Assign tracking ids to the skeletons in the present frame
            CHECK_SUCCESS(cm_skel_update_tracking(handle, &imagePresent, tracking_pipeline.get(), skeletons.get()));
            if (skeletons->numSkeletons > 0) {
                // Render skeleton overlays with tracking ids
                renderSkeletons(skeletons.get(), depthFrame, capturedFrame);
            }
        }

        frameCount++;
        if (frameCount % 25 == 0) {
            auto timePassed =
              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime)
                .count();
            auto fps = 25000.0 / timePassed;

            fpsTest = "Frame rate: " + std::to_string(fps) + " FPS";
            startTime = std::chrono::system_clock::now();
        }
        cv::putText(capturedFrame, fpsTest, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, skeletonColor);

        cv::imshow(cvWindowName, capturedFrame);
		cv::imwrite("dana_cubemos_FINAL.png",NULL);
    }

    // release the memory which is managed by the cubemos framework
    cm_skel_destroy_handle(&handle);
    return 0;
}
