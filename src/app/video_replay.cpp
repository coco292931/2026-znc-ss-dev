#include "path_controller.hpp"
#include "vision_pipeline.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <opencv2/videoio.hpp>

using namespace rewrite_path;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "Usage: %s INPUT_VIDEO OUTPUT_EVENTS.csv\n", argv[0]);
        return 2;
    }

    cv::VideoCapture capture(argv[1]);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "failed to open video: %s\n", argv[1]);
        return 1;
    }
    std::FILE* output = std::fopen(argv[2], "wb");
    if (!output) {
        std::perror("fopen");
        return 1;
    }

    double fps = capture.get(cv::CAP_PROP_FPS);
    if (!(fps > 1.0 && fps < 240.0)) fps = 30.0;

    PathParams params;
    params.dry_run = true;
    params.enable_side_road = true;
    params.enable_target_actions = false;
    params.camera_fps = static_cast<int>(std::lround(fps));
    LegacyVisionPipeline vision(params);
    PathController navigation(params);

    const double dt = 1.0 / fps;
    double distance_cm = 0.0;
    double heading_deg = 0.0;
    int frame_index = 0;
    DriveState previous_state = DriveState::Follow;
    std::fprintf(
        output,
        "frame,time_s,line_error,far_error,confidence,line_lost,cross,"
        "zebra,zebra_active,zebra_encounter,"
        "side_open,roundabout_stage,stable_side,left_branches,right_branches,"
        "left_recoveries,right_recoveries,state,state_changed,"
        "target_speed_cmps,target_yaw_rate_dps\n");

    cv::Mat frame;
    while (capture.read(frame)) {
        if (params.rotate_180) cv::rotate(frame, frame, cv::ROTATE_180);
        const RoadEstimateLite road = vision.process_bgr(frame);
        StepInput input;
        input.line_error = road.line_error;
        input.far_error = road.far_error;
        input.line_confidence = road.line_confidence;
        input.line_lost = road.line_lost;
        input.cross = road.elements.cross;
        input.zebra = road.elements.zebra;
        input.side_open = road.elements.side_open;
        input.roundabout = road.elements.roundabout;
        input.roundabout_stage = road.elements.roundabout_stage;
        input.two_side_stable = road.elements.two_side_stable;
        input.distance_cm = distance_cm;
        input.distance_valid = true;
        input.encoder_heading_deg = heading_deg;
        input.encoder_heading_valid = true;
        // Replay has no physical IMU; mark the synthetic motion source as
        // qualified so side-road state transitions remain testable offline.
        input.heading_deg = heading_deg;
        input.heading_valid = true;
        input.heading_from_imu = true;
        const NavigationCommand command = navigation.update(input, dt);
        distance_cm += command.target_speed_cmps * dt;
        heading_deg += command.target_yaw_rate_dps * dt;
        const bool changed = command.state != previous_state;
        std::fprintf(
            output,
            "%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%s,%s,%s,%d,%d,%d,%d,%s,%d,%.6f,%.6f\n",
            frame_index,
            frame_index * dt,
            road.line_error,
            road.far_error,
            road.line_confidence,
            road.line_lost ? 1 : 0,
            road.elements.cross ? 1 : 0,
            road.elements.zebra ? 1 : 0,
            command.zebra_active ? 1 : 0,
            command.zebra_encounter_count,
            feature_side_name(road.elements.side_open),
            roundabout_stage_name(road.elements.roundabout_stage),
            feature_side_name(road.elements.stable_side),
            road.elements.left_branch_count,
            road.elements.right_branch_count,
            road.elements.left_recovery_count,
            road.elements.right_recovery_count,
            drive_state_name(command.state),
            changed ? 1 : 0,
            command.target_speed_cmps,
            command.target_yaw_rate_dps);
        previous_state = command.state;
        ++frame_index;
    }
    std::fclose(output);
    std::printf("processed %d frames -> %s\n", frame_index, argv[2]);
    return frame_index > 0 ? 0 : 1;
}


