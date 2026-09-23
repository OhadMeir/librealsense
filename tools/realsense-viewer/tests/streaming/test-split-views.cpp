// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// A stream whose frames come in two kinds is shown as two tiles, each fed by its own post-processing
// chain. The split is decided from per-frame metadata, so only a real device exercises it - today that
// means Alternating Passive Depth on a D585 2C, where the firmware tags laser-on and laser-off frames.

#include "viewer-test-helpers.h"


namespace {

// Dual-RGB only: the two color streams share the depth sensor's imagers, which is what makes the
// firmware interleave the exposures, and the control is registered nowhere else. Every 2C SKU carries
// "Dual RGB" in its name, so the name says what the test is limited to and the control says whether
// this firmware actually offers it.
std::shared_ptr< rs2::subdevice_model > find_splitting_sensor( rs2::device_model & model )
{
    if( ! model.dev.supports( RS2_CAMERA_INFO_NAME ) )
        return {};
    if( std::string( model.dev.get_info( RS2_CAMERA_INFO_NAME ) ).find( "Dual RGB" ) == std::string::npos )
        return {};

    for( auto && sub : model.subdevices )
        if( sub->s->supports( RS2_OPTION_PASSIVE_DEPTH_MODE ) )
            return sub;
    return {};
}

// Frame numbers of both tiles of every split stream, empty while nothing is split
std::map< int, unsigned long long > view_frame_numbers( rs2::viewer_model & viewer )
{
    std::map< int, unsigned long long > numbers;
    std::lock_guard< std::mutex > lock( viewer.streams_mutex );
    for( auto && split : viewer.split_views )
        for( int key : { split.first, split.second } )
        {
            auto it = viewer.streams.find( key );
            if( it != viewer.streams.end() )
                numbers[key] = it->second.frame_number;
        }
    return numbers;
}

}  // namespace


VIEWER_TEST( "streaming", "split_views_each_get_their_own_frames" )
{
    auto & model = test.find_first_device_or_exit();

    auto sub = find_splitting_sensor( model );
    if( ! sub )
        return;  // not a dual-RGB model, or firmware without the control - nothing splits here

    // The suite shares one viewer, so a test that failed midway can leave a sensor streaming. Start from
    // stopped rather than inheriting that - the mode cannot be set while streaming anyway.
    for( auto && other : model.subdevices )
        if( other->streaming )
            test.click_stream_toggle_off( model, other );

    test.expand_sensor_panel( model, sub );
    test.expand_controls( model, sub );
    test.set_control_value( model, sub, RS2_OPTION_PASSIVE_DEPTH_MODE, "Alternating" );

    test.click_stream_toggle_on( model, sub );

    // Every filter the sensor recommends must have a second instance, or the two kinds of frame would
    // still be sharing a temporal history
    for( auto && pb : sub->post_processing )
        IM_CHECK( pb->has_secondary() );

    // The split is created at stream-on, from the mode the sensor reports
    IM_CHECK( test.wait_until( 40, 0.25f, [&]()
                               {
                                   std::lock_guard< std::mutex > lock( test.viewer_model.streams_mutex );
                                   return ! test.viewer_model.split_views.empty();
                               } ) );

    // Both tiles have to be fed: the firmware only tags the frames while a color stream shares the
    // imagers, so a stream set without color leaves the secondary tile empty
    IM_CHECK( test.wait_until( 40, 0.25f, [&]()
                               {
                                   for( auto && view : view_frame_numbers( test.viewer_model ) )
                                       if( ! view.second )
                                           return false;
                                   return true;
                               } ) );

    auto before = view_frame_numbers( test.viewer_model );
    test.sleep( 2.0f );
    auto after = view_frame_numbers( test.viewer_model );

    IM_CHECK( ! before.empty() );
    for( auto && view : before )
    {
        IM_CHECK( after.count( view.first ) );
        IM_CHECK( after[view.first] > view.second );  // still arriving, not one stale frame
    }

    {
        std::lock_guard< std::mutex > lock( test.viewer_model.streams_mutex );
        for( auto && split : test.viewer_model.split_views )
        {
            IM_CHECK_STR_EQ( test.viewer_model.streams[split.first].view_label.c_str(), "Active" );
            IM_CHECK_STR_EQ( test.viewer_model.streams[split.second].view_label.c_str(), "Passive" );
            IM_CHECK( test.viewer_model.streams[split.second].secondary_view );
        }
    }

    test.click_stream_toggle_off( model, sub );
    test.sleep( 1.0f );
    test.set_control_value( model, sub, RS2_OPTION_PASSIVE_DEPTH_MODE, "Disabled" );
}
