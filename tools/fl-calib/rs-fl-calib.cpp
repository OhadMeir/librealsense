// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include <common/cli.h>
#include <rsutils/string/from.h>


static const double pi = std::acos(-1);
static const double d2r = pi / 180;
static const double r2d = 180 / pi;
template<typename T> T deg2rad(T val) { return T(val * d2r); }
template<typename T> T rad2deg(T val) { return T(val * r2d); }

void load_frames( const std::string& path, rs2::frame_queue& left_queue, rs2::frame_queue& right_queue, size_t number_of_frames)
{
    std::cout << "Loading frames ... ";

    auto pipe = std::make_shared< rs2::pipeline >();
    rs2::config cfg;
    cfg.enable_device_from_file( path, false ); // Do not loop over the file
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    pipe->start( cfg ); //File will be opened in read mode at this point
    rs2::device device = pipe->get_active_profile().get_device();

    rs2::frameset frames;
    rs2::frame frame;

    if( rs2::playback playback = device.as< rs2::playback >() )
    {
        // Read until end of file or requested number of frames, the earlier.
        for( size_t i = 0; i < number_of_frames; ++i ) // Check if new frames are ready
        {
            if( frames = pipe->wait_for_frames( ) )
            {
                frame = frames.get_infrared_frame( 1 );
                left_queue.enqueue( frame );

                frame = frames.get_infrared_frame( 2 );
                right_queue.enqueue( frame );
            }
        }
    }

    pipe->stop();

    if( left_queue.size() < number_of_frames || right_queue.size() < number_of_frames )
    {
        throw std::runtime_error( rsutils::string::from() << "Failed to load frames! Got " <<
                                  left_queue.size() << " left IR frames, " << right_queue.size() << " right IR frames" );
    }

    std::cout << "done." << std::endl;
}

void get_target_rect_info( rs2_frame_queue * frames, float rect_sides[4], float & fx, float & fy )
{
    rs2_error * e = nullptr;
    int queue_size = rs2_frame_queue_size( frames, &e );
    if( queue_size == 0 )
        throw std::runtime_error( "Extract target rectangle info - no frames in input queue!" );

    int frame_counter = 0;
    bool first_time = true;
    rs2_frame * f = nullptr;
    std::vector< std::array< float, 4 > > rect_sides_arr;
    while( ( frame_counter++ < queue_size ) && rs2_poll_for_frame( frames, &f, &e ) )
    {
        std::cout << frame_counter << ", ";
        rs2::frame ff( f );
        if( ff.get_data() )
        {
            if( first_time )
            {
                auto p = ff.get_profile();
                auto vsp = p.as< rs2::video_stream_profile >();
                rs2_intrinsics intrin = vsp.get_intrinsics();
                fx = intrin.fx;
                fy = intrin.fy;
                first_time = false;
            }

            std::array< float, 4 > rec_sides_cur{};
            rs2_extract_target_dimensions( f,
                                           RS2_CALIB_TARGET_ROI_RECT_GAUSSIAN_DOT_VERTICES,
                                           rec_sides_cur.data(),
                                           4,
                                           &e );
            if( e )
                throw std::runtime_error( "Failed to extract target information\nfrom the captured frames!" );
            rect_sides_arr.emplace_back( rec_sides_cur );
        }

        ff = {}; // Release the frame, don't need it for progress callback
    }

    if( rect_sides_arr.size() )
    {
        for( int i = 0; i < 4; ++i )
            rect_sides[i] = rect_sides_arr[0][i];

        for( int j = 1; j < rect_sides_arr.size(); ++j )
        {
            for( int i = 0; i < 4; ++i )
                rect_sides[i] += rect_sides_arr[j][i];
        }

        for( int i = 0; i < 4; ++i )
            rect_sides[i] /= rect_sides_arr.size();
    }
    else
        throw std::runtime_error( "Failed to extract the target rectangle info!" );
}

float get_focal_length_correction_factor( const float left_rect_sides[4],
                                          const float right_rect_sides[4],
                                          const float fx[2],
                                          const float fy[2],
                                          const float target_w,
                                          const float target_h,
                                          const float baseline,
                                          float & ratio,
                                          float & angle )
{
    const float correction_factor = 0.5f;

    float ar[2] = { 0 };
    float tmp = left_rect_sides[2] + left_rect_sides[3];
    if( tmp > 0.1f )
        ar[0] = ( left_rect_sides[0] + left_rect_sides[1] ) / tmp;

    tmp = right_rect_sides[2] + right_rect_sides[3];
    if( tmp > 0.1f )
        ar[1] = ( right_rect_sides[0] + right_rect_sides[1] ) / tmp;

    float align = 0.0f;
    if( ar[0] > 0.0f )
        align = ar[1] / ar[0] - 1.0f;

    float ta[2] = { 0 };
    float gt[4] = { 0 };
    float ave_gt = 0.0f;

    if( left_rect_sides[0] > 0 )
        gt[0] = fx[0] * target_w / left_rect_sides[0];

    if( left_rect_sides[1] > 0 )
        gt[1] = fx[0] * target_w / left_rect_sides[1];

    if( left_rect_sides[2] > 0 )
        gt[2] = fy[0] * target_h / left_rect_sides[2];

    if( left_rect_sides[3] > 0 )
        gt[3] = fy[0] * target_h / left_rect_sides[3];

    ave_gt = 0.0f;
    for( int i = 0; i < 4; ++i )
        ave_gt += gt[i];
    ave_gt /= 4.0;

    ta[0] = atanf( align * ave_gt / std::abs( baseline ) );
    ta[0] = rad2deg( ta[0] );

    if( right_rect_sides[0] > 0 )
        gt[0] = fx[1] * target_w / right_rect_sides[0];

    if( right_rect_sides[1] > 0 )
        gt[1] = fx[1] * target_w / right_rect_sides[1];

    if( right_rect_sides[2] > 0 )
        gt[2] = fy[1] * target_h / right_rect_sides[2];

    if( right_rect_sides[3] > 0 )
        gt[3] = fy[1] * target_h / right_rect_sides[3];

    ave_gt = 0.0f;
    for( int i = 0; i < 4; ++i )
        ave_gt += gt[i];
    ave_gt /= 4.0;

    ta[1] = atanf( align * ave_gt / std::abs( baseline ) );
    ta[1] = rad2deg( ta[1] );

    angle = ( ta[0] + ta[1] ) / 2;

    align *= 100;

    float r[4] = { 0 };
    float c = fx[0] / fx[1];

    if( left_rect_sides[0] > 0.1f )
        r[0] = c * right_rect_sides[0] / left_rect_sides[0];

    if( left_rect_sides[1] > 0.1f )
        r[1] = c * right_rect_sides[1] / left_rect_sides[1];

    c = fy[0] / fy[1];
    if( left_rect_sides[2] > 0.1f )
        r[2] = c * right_rect_sides[2] / left_rect_sides[2];

    if( left_rect_sides[3] > 0.1f )
        r[3] = c * right_rect_sides[3] / left_rect_sides[3];

    float ra = 0.0f;
    for( int i = 0; i < 4; ++i )
        ra += r[i];
    ra /= 4;

    ra -= 1.0f;
    ra *= 100;

    ratio = ra - correction_factor * align;
    float ratio_to_apply = ratio / 100.0f + 1.0f;

    return ratio_to_apply;
}

// TODO - read a calibration table from disk and return updated table
void run_focal_length_calibration( rs2_frame_queue * left, rs2_frame_queue * right,
                                   float target_w, float target_h, float baseline,
                                   float & ratio, float & angle)
{
    float fx[2] = { -1.0f, -1.0f };
    float fy[2] = { -1.0f, -1.0f };

    std::cout << "Getting target rect info for left frames ... ";
    float left_rect_sides[4] = {0.f};
    get_target_rect_info( left, left_rect_sides, fx[0], fy[0] );
    std::cout << "done." << std::endl;

    std::cout << "Getting target rect info for right frames ... ";
    float right_rect_sides[4] = {0.f};
    get_target_rect_info( right, right_rect_sides, fx[1], fy[1]);
    std::cout << "done." << std::endl;

    std::cout << "Calculating correction factor ... ";
    get_focal_length_correction_factor( left_rect_sides, right_rect_sides, fx, fy,
                                        target_w, target_h, baseline, ratio, angle );
    std::cout << "done. ratio = " << ratio << " angle = " << angle << std::endl;
}

int main(int argc, char * argv[]) try
{
    using rs2::cli;
    cli cmd( "rs-fl-calib example" );
    cli::value< std::string > path_arg('p', "path", "path", "", "Path of the folder containing the files");
    cli::value< unsigned int > number_arg( 'n', "number", "unsigned", 25, "Number of frames per channel" );
    cli::value< float > width_arg( 'w', "target_width", "float", 175.0f, "Target width, millimeters" );
    cli::value< float > height_arg( 'h', "target_height", "float", 100.0f, "Target height, millimeters" );
    cli::value< float > baseline_arg( 'b', "baseline", "float", 100.0f, "baseline, millimeters" );
    cmd.add(path_arg);
    auto settings = cmd.process( argc, argv );

    if( !path_arg.isSet() )
    {
        std::cout << "Nothing to do, run again with -h for help" << std::endl;
        return EXIT_FAILURE;
    }

    rs2::context ctx( settings.dump() );

    rs2::frame_queue left_queue( number_arg.getValue() );
    rs2::frame_queue right_queue( number_arg.getValue() );
    load_frames( path_arg.getValue(), left_queue, right_queue, number_arg.getValue() );

    float ratio = 0.0f;
    float angle = 0.0f;
    run_focal_length_calibration( left_queue.get().get(), right_queue.get().get(),
                                  width_arg.getValue(), height_arg.getValue(), baseline_arg.getValue(),
                                  ratio, angle );

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
