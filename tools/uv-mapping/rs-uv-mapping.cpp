// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

#include <common/cli.h>
#include <rsutils/string/from.h>

#include <thread>

static const double pi = std::acos(-1);
static const double d2r = pi / 180;
static const double r2d = 180 / pi;
template<typename T> T deg2rad(T val) { return T(val * d2r); }
template<typename T> T rad2deg(T val) { return T(val * r2d); }

void load_data( const std::string & path,
                rs2::frame_queue & infra_queue,
                rs2::frame_queue & color_queue,
                rs2::frame_queue & depth_queue,
                rs2_extrinsics & extr,
                size_t number_of_frames = 0, unsigned int from = 0, unsigned int to = 0)
{
    if( from != 0 && to != 0 )
        number_of_frames = to - from;

    if( number_of_frames == 0 )
        throw std::invalid_argument( "Requested number of frames is 0!" );

    std::cout << "Loading frames ... " << std::endl;

    auto pipe = std::make_shared< rs2::pipeline >();
    rs2::config cfg;
    cfg.enable_stream( RS2_STREAM_INFRARED, 1, 1280, 720 );
    cfg.enable_stream( RS2_STREAM_COLOR, 1280, 720 );
    cfg.enable_stream( RS2_STREAM_DEPTH, 1280, 720 );
    cfg.enable_device_from_file( path, false ); // Do not loop over the file

    pipe->start( cfg );

    rs2::pipeline_profile prof = pipe->get_active_profile();
    rs2::stream_profile infra_stream = prof.get_stream( RS2_STREAM_INFRARED );
    rs2::stream_profile color_stream = prof.get_stream( RS2_STREAM_COLOR );
    extr = infra_stream.get_extrinsics_to( color_stream );

    rs2::frameset frames;
    rs2::frame infra_frame;
    rs2::frame color_frame;
    rs2::frame depth_frame;
    size_t counter = 0;

    // Read until end of file or requested number of frames, the earlier.
    while( frames = pipe->wait_for_frames( 100 ) )
    {
        infra_frame = frames.get_infrared_frame( 1 );
        if( to > 0 && infra_frame.get_frame_number() < from )  // Use frame range
            continue;

        color_frame = frames.get_color_frame();
        depth_frame = frames.get_depth_frame();

        std::cout << "Got frame " << infra_frame.get_frame_number() << std::endl;

        infra_queue.enqueue( infra_frame );
        color_queue.enqueue( color_frame );
        depth_queue.enqueue( depth_frame );

        if( ++counter == number_of_frames )
            break;
    }

    pipe->stop();

    if( infra_queue.size() < number_of_frames || color_queue.size() < number_of_frames || depth_queue.size() < number_of_frames )
    {
        throw std::runtime_error( rsutils::string::from() << "Failed to load frames! Got " <<
                                  infra_queue.size() << " IR frames, " << color_queue.size() << " color frames, " <<
                                  depth_queue.size() << " depth frames" );
    }

    std::cout << "done." << std::endl;
}

void undistort( uint8_t * img, const rs2_intrinsics & intrin, int roi_ws, int roi_hs, int roi_we, int roi_he )
{
    assert( intrin.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY );

    int width = intrin.width;
    int height = intrin.height;

    if( roi_ws < 0 )
        roi_ws = 0;
    if( roi_hs < 0 )
        roi_hs = 0;
    if( roi_we > width )
        roi_we = width;
    if( roi_he > height )
        roi_he = height;

    int size3 = width * height * 3;
    std::vector< uint8_t > tmp( size3 );
    memset( tmp.data(), 0, size3 );

    float x = 0;
    float y = 0;
    int m = 0;
    int n = 0;

    int width3 = width * 3;
    int idx_from = 0;
    int idx_to = 0;
    for( int j = roi_hs; j < roi_he; ++j )
    {
        for( int i = roi_ws; i < roi_we; ++i )
        {
            x = static_cast< float >( i );
            y = static_cast< float >( j );

            if( std::abs( intrin.fx ) > 0.00001f && std::abs( intrin.fy ) > 0.0001f )
            {
                x = ( x - intrin.ppx ) / intrin.fx;
                y = ( y - intrin.ppy ) / intrin.fy;

                float r2 = x * x + y * y;
                float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
                float ux = x * f + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * ( r2 + 2 * x * x );
                float uy = y * f + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * ( r2 + 2 * y * y );
                x = ux;
                y = uy;

                x = x * intrin.fx + intrin.ppx;
                y = y * intrin.fy + intrin.ppy;
            }

            m = static_cast< int >( x + 0.5f );
            if( m >= 0 && m < width )
            {
                n = static_cast< int >( y + 0.5f );
                if( n >= 0 && n < height )
                {
                    idx_from = j * width3 + i * 3;
                    idx_to = n * width3 + m * 3;
                    tmp[idx_to++] = img[idx_from++];
                    tmp[idx_to++] = img[idx_from++];
                    tmp[idx_to++] = img[idx_from++];
                }
            }
        }
    }

    memmove( img, tmp.data(), size3 );
}

void get_target_dots_info( rs2_frame_queue * frames, float dots_x[4], float dots_y[4], rs2::stream_profile & profile,
                           rs2_intrinsics & intrin, int progress, std::function< void( float ) > progress_callback )
{
    bool got_intrinsics = false;
    std::vector< std::array< float, 4 > > dots_x_arr;
    std::vector< std::array< float, 4 > > dots_y_arr;

    rs2_error * e = nullptr;
    rs2_frame * f = nullptr;

    int queue_size = rs2_frame_queue_size( frames, &e );
    int fc = 0;

    while( ( fc++ < queue_size ) && rs2_poll_for_frame( frames, &f, &e ) )
    {
        rs2::frame ff( f );
        if( ff.get_data() )
        {
            if( ! got_intrinsics )
            {
                profile = ff.get_profile();
                auto vsp = profile.as< rs2::video_stream_profile >();
                intrin = vsp.get_intrinsics();
                got_intrinsics = true;
            }

            if( ff.get_profile().format() == RS2_FORMAT_RGB8 )
            {
                constexpr const int _roi_ws = 480;
                constexpr const int _roi_we = 800;
                constexpr const int _roi_hs = 240;
                constexpr const int _roi_he = 480;
                constexpr const int _patch_size = 20;  // in pixels
                undistort( const_cast< uint8_t * >( static_cast< const uint8_t * >( ff.get_data() ) ), intrin,
                           _roi_ws - _patch_size, _roi_hs - _patch_size, _roi_we + _patch_size, _roi_he + _patch_size );
            }

            float dots[8] = { 0 };
            rs2_extract_target_dimensions( f, RS2_CALIB_TARGET_POS_GAUSSIAN_DOT_VERTICES, dots, 8, &e );
            if( e )
                throw std::runtime_error( "Failed to extract target information\nfrom the captured frames!" );

            std::array< float, 4 > dots_x_cur;
            std::array< float, 4 > dots_y_cur;
            int j = 0;
            for( int i = 0; i < 4; ++i )
            {
                j = i << 1;
                dots_x_cur[i] = dots[j];
                dots_y_cur[i] = dots[j + 1];
            }
            dots_x_arr.emplace_back( dots_x_cur );
            dots_y_arr.emplace_back( dots_y_cur );
        }

        ff = {};  // Release the frame

        if( progress_callback )
            progress_callback( static_cast< float >( ++progress ) );
    }

    for( int i = 0; i < 4; ++i )
    {
        dots_x[i] = dots_x_arr[0][i];
        dots_y[i] = dots_y_arr[0][i];
    }

    for( int j = 1; j < dots_x_arr.size(); ++j )
    {
        for( int i = 0; i < 4; ++i )
        {
            dots_x[i] += dots_x_arr[j][i];
            dots_y[i] += dots_y_arr[j][i];
        }
    }

    for( int i = 0; i < 4; ++i )
    {
        dots_x[i] /= dots_x_arr.size();
        dots_y[i] /= dots_y_arr.size();
    }
}

void find_z_at_corners( float left_x[4], float left_y[4], rs2_frame_queue * frames, float left_z[4] )
{
    int x1[4] = { 0 };
    int y1[4] = { 0 };
    int x2[4] = { 0 };
    int y2[4] = { 0 };
    int pos_tl[4] = { 0 };
    int pos_tr[4] = { 0 };
    int pos_bl[4] = { 0 };
    int pos_br[4] = { 0 };

    float left_z_tl[4] = { 0 };
    float left_z_tr[4] = { 0 };
    float left_z_bl[4] = { 0 };
    float left_z_br[4] = { 0 };

    bool got_width = false;
    int width = 0;
    int counter = 0;
    rs2_error * e = nullptr;
    rs2_frame * f = nullptr;

    int queue_size = rs2_frame_queue_size( frames, &e );
    int fc = 0;

    while( ( fc++ < queue_size ) && rs2_poll_for_frame( frames, &f, &e ) )
    {
        rs2::frame ff( f );
        if( ff.get_data() )
        {
            if( ! got_width )
            {
                auto p = ff.get_profile();
                auto vsp = p.as< rs2::video_stream_profile >();
                width = vsp.get_intrinsics().width;
                got_width = true;

                for( int i = 0; i < 4; ++i )
                {
                    x1[i] = static_cast< int >( left_x[i] );
                    y1[i] = static_cast< int >( left_y[i] );

                    x2[i] = static_cast< int >( left_x[i] + 1.0f );
                    y2[i] = static_cast< int >( left_y[i] + 1.0f );

                    pos_tl[i] = y1[i] * width + x1[i];
                    pos_tr[i] = y1[i] * width + x2[i];
                    pos_bl[i] = y2[i] * width + x1[i];
                    pos_br[i] = y2[i] * width + x2[i];
                }
            }

            const uint16_t * depth = reinterpret_cast< const uint16_t * >( ff.get_data() );
            for( int i = 0; i < 4; ++i )
            {
                left_z_tl[i] += static_cast< float >( depth[pos_tl[i]] );
                left_z_tr[i] += static_cast< float >( depth[pos_tr[i]] );
                left_z_bl[i] += static_cast< float >( depth[pos_bl[i]] );
                left_z_br[i] += static_cast< float >( depth[pos_br[i]] );
            }

            ++counter;
        }

        ff = {}; // Release the frame
    }

    for( int i = 0; i < 4; ++i )
    {
        if( counter > 0 )
        {
            left_z_tl[i] /= counter;
            left_z_tr[i] /= counter;
            left_z_bl[i] /= counter;
            left_z_br[i] /= counter;
        }
    }

    float z_1 = 0.0f;
    float z_2 = 0.0f;
    float s = 0.0f;
    for( int i = 0; i < 4; ++i )
    {
        s = left_x[i] - x1[i];
        z_1 = ( 1.0f - s ) * left_z_bl[i] + s * left_z_br[i];
        z_2 = ( 1.0f - s ) * left_z_tl[i] + s * left_z_tr[i];

        s = left_y[i] - y1[i];
        left_z[i] = ( 1.0f - s ) * z_2 + s * z_1;
        left_z[i] *= 0.001f;
    }
}

void run_uv_map_calibration( rs2_frame_queue * left, rs2_frame_queue * color, rs2_frame_queue * depth, bool py_px_only, const rs2_extrinsics & extr,
                             float * const health, int health_size, std::function< void( float ) > progress_callback )
{
    float left_dots_x[4];
    float left_dots_y[4];
    rs2_intrinsics left_intrin;
    rs2::stream_profile left_profile;
    get_target_dots_info( left, left_dots_x, left_dots_y, left_profile, left_intrin, 50, progress_callback );

    float color_dots_x[4];
    float color_dots_y[4];
    rs2_intrinsics color_intrin;
    rs2::stream_profile color_profile;
    get_target_dots_info( color, color_dots_x, color_dots_y, color_profile, color_intrin, 75, progress_callback );

    float z[4] = { 0 };
    find_z_at_corners( left_dots_x, left_dots_y, depth, z );

    float pixel_left[4][2] = { 0 };
    float point_left[4][3] = { 0 };

    float pixel_color[4][2] = { 0 };
    float pixel_color_norm[4][2] = { 0 };
    float point_color[4][3] = { 0 };

    for( int i = 0; i < 4; ++i )
    {
        pixel_left[i][0] = left_dots_x[i];
        pixel_left[i][1] = left_dots_y[i];

        rs2_deproject_pixel_to_point( point_left[i], &left_intrin, pixel_left[i], z[i] );
        rs2_transform_point_to_point( point_color[i], &extr, point_left[i] );

        pixel_color_norm[i][0] = point_color[i][0] / point_color[i][2];
        pixel_color_norm[i][1] = point_color[i][1] / point_color[i][2];
        pixel_color[i][0] = pixel_color_norm[i][0] * color_intrin.fx + color_intrin.ppx;
        pixel_color[i][1] = pixel_color_norm[i][1] * color_intrin.fy + color_intrin.ppy;
    }

    float diff[4] = { 0 };
    float tmp = 0.0f;
    for( int i = 0; i < 4; ++i )
    {
        tmp = ( pixel_color[i][0] - color_dots_x[i] );
        tmp *= tmp;
        diff[i] = tmp;

        tmp = ( pixel_color[i][1] - color_dots_y[i] );
        tmp *= tmp;
        diff[i] += tmp;

        diff[i] = sqrtf( diff[i] );
    }

    float err_before = 0.0f;
    for( int i = 0; i < 4; ++i )
        err_before += diff[i];
    err_before /= 4;

    float ppx = 0.0f;
    float ppy = 0.0f;
    float fx = 0.0f;
    float fy = 0.0f;

    if( py_px_only )
    {
        fx = color_intrin.fx;
        fy = color_intrin.fy;

        ppx = 0.0f;
        ppy = 0.0f;
        for( int i = 0; i < 4; ++i )
        {
            ppx += color_dots_x[i] - pixel_color_norm[i][0] * fx;
            ppy += color_dots_y[i] - pixel_color_norm[i][1] * fy;
        }
        ppx /= 4.0f;
        ppy /= 4.0f;
    }
    else
    {
        double x = 0;
        double y = 0;
        double c_x = 0;
        double c_y = 0;
        double x_2 = 0;
        double y_2 = 0;
        double c_xc = 0;
        double c_yc = 0;
        for( int i = 0; i < 4; ++i )
        {
            x += pixel_color_norm[i][0];
            y += pixel_color_norm[i][1];
            c_x += color_dots_x[i];
            c_y += color_dots_y[i];
            x_2 += pixel_color_norm[i][0] * pixel_color_norm[i][0];
            y_2 += pixel_color_norm[i][1] * pixel_color_norm[i][1];
            c_xc += color_dots_x[i] * pixel_color_norm[i][0];
            c_yc += color_dots_y[i] * pixel_color_norm[i][1];
        }

        double d_x = 4 * x_2 - x * x;
        if( d_x > 0.01 )
        {
            d_x = 1 / d_x;
            fx = static_cast< float >( d_x * ( 4 * c_xc - x * c_x ) );
            ppx = static_cast< float >( d_x * ( x_2 * c_x - x * c_xc ) );
        }

        double d_y = 4 * y_2 - y * y;
        if( d_y > 0.01 )
        {
            d_y = 1 / d_y;
            fy = static_cast< float >( d_y * ( 4 * c_yc - y * c_y ) );
            ppy = static_cast< float >( d_y * ( y_2 * c_y - y * c_yc ) );
        }
    }

    float err_after = 0.0f;
    float tmpx = 0;
    float tmpy = 0;
    for( int i = 0; i < 4; ++i )
    {
        tmpx = pixel_color_norm[i][0] * fx + ppx - color_dots_x[i];
        tmpx *= tmpx;

        tmpy = pixel_color_norm[i][1] * fy + ppy - color_dots_y[i];
        tmpy *= tmpy;

        err_after += sqrtf( tmpx + tmpy );
    }

    err_after /= 4.0f;

    std::vector< uint8_t > ret;
    const float max_change = 16.0f;
    if( fabs( color_intrin.ppx - ppx ) < max_change && fabs( color_intrin.ppy - ppy ) < max_change &&
        fabs( color_intrin.fx - fx ) < max_change && fabs( color_intrin.fy - fy ) < max_change )
    {
        health[0] = ppx;
        health[1] = ppy;
        health[2] = fx;
        health[3] = fy;
    }
}


int main(int argc, char * argv[]) try
{
    using rs2::cli;
    cli cmd( "rs-uv-mapping tool" );
    cli::value< std::string > path_arg('p', "path", "path", "", "Path of the folder containing the files");
    cli::value< unsigned int > number_arg( 'n', "number", "unsigned", 25, "Number of frames per channel" );
    cli::value< unsigned int > from_arg( 'f', "from", "unsigned", 0, "When specifying frame range - first IR frame in the range" );
    cli::value< unsigned int > to_arg( 't', "to", "unsigned", 0, "When specifying frame range - last IR frame in the range" );
    cli::value< float > width_arg( "target_width", "float", 175.0f, "Target width, millimeters" );
    cli::value< float > height_arg( "target_height", "float", 100.0f, "Target height, millimeters" );
    cli::value< int > py_px_only_arg( "py_px_only", "int", 1, "Fix py and px only. Default 1 (true)" );
    cmd.add( path_arg );
    cmd.add( number_arg );
    cmd.add( from_arg );
    cmd.add( to_arg );
    cmd.add( width_arg );
    cmd.add( height_arg );
    cmd.add( py_px_only_arg );
    auto settings = cmd.process( argc, argv );

    if( !path_arg.isSet() )
    {
        std::cout << "Nothing to do, run again with -h for help" << std::endl;
        return EXIT_FAILURE;
    }

    rs2::context ctx( settings.dump() );

    rs2::frame_queue infra_queue( number_arg.getValue() );
    rs2::frame_queue color_queue( number_arg.getValue() );
    rs2::frame_queue depth_queue( number_arg.getValue() );
    rs2_extrinsics extr;
    load_data( path_arg.getValue(), infra_queue, color_queue, depth_queue, extr, number_arg.getValue(), from_arg.getValue(), to_arg.getValue() );

    std::cout << "Calculating correction factor ... " << std::endl;
    constexpr const int health_size = 4;
    float health[health_size];
    run_uv_map_calibration( infra_queue.get().get(), color_queue.get().get(), depth_queue.get().get(), py_px_only_arg.getValue(), extr,
                            health, health_size, []( float progress ) { std::cout << "Progress: " << progress << std::endl; } );
    std::cout << "... done. ppx = " << health[0] << " ppy = " << health[1] << " fx = " << health[2] << " fy = " << health[3] << std::endl;

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
