// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "h264-decoder.h"

#include <rsutils/easylogging/easyloggingpp.h>

namespace librealsense
{
#ifndef _WIN32
    std::unique_ptr< h264_decoder_backend > h264_decoder_backend::create( int, int, int )
    {
        return nullptr;
    }
#endif

    bool h264_decoder::is_supported()
    {
        // Probed once with an arbitrary small profile - creating a decoder is what tells us whether
        // the platform has one at all
        static bool const supported = h264_decoder_backend::create( 640, 360, 30 ) != nullptr;
        return supported;
    }

    rs2::frame h264_decoder::process_frame( const rs2::frame_source & source, const rs2::frame & f )
    {
        auto vf = f.as< rs2::video_frame >();
        if( ! vf )
            return {};

        if( ! _backend || vf.get_width() != _width || vf.get_height() != _height )
        {
            _width = vf.get_width();
            _height = vf.get_height();
            _backend = h264_decoder_backend::create( _width, _height, f.get_profile().fps() );
            if( ! _backend )
                return {};
        }

        if( ! _backend->decode( static_cast< uint8_t const * >( f.get_data() ), f.get_data_size(), _nv12 ) )
            return {};

        auto ret = prepare_frame( source, f );  // also initializes _target_bpp
        uint8_t * planes[1] = { const_cast< uint8_t * >( static_cast< uint8_t const * >( ret.get_data() ) ) };
        process_function( planes, nullptr, _width, _height, _height * _width * _target_bpp, 0 );
        return ret;
    }

    void h264_decoder::process_function( uint8_t * const dest[], const uint8_t *, int width, int height,
                                         int actual_size, int )
    {
        // Source is the decoded NV12, not the bitstream we were handed
        unpack_nv12( _target_format, _target_stream, dest, _nv12.data(), width, height, actual_size );
    }

    rs2::frame h264_decoder::prepare_output( const rs2::frame_source & source,
                                             rs2::frame input,
                                             std::vector< rs2::frame > results )
    {
        // The base implementation falls back to the input frame, which here is the undecoded
        // bitstream - it must not reach the application
        if( results.empty() )
            return {};
        return color_converter::prepare_output( source, input, results );
    }
}
