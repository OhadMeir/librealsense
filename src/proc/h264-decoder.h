// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include "color-formats-converter.h"

#include <memory>
#include <vector>
#include <stdint.h>

namespace librealsense
{
    // Platform H.264 decoder. Takes one Annex-B access unit at a time and produces tightly packed
    // NV12 at the requested resolution. A single instance is only ever used from one thread.
    class h264_decoder_backend
    {
    public:
        virtual ~h264_decoder_backend() = default;

        // Decodes one access unit into 'nv12' (width*height*3/2 bytes). Returns false when no
        // picture is available yet: the decoder is priming, or waiting for parameter sets after a
        // loss. Callers should drop the frame rather than render stale content.
        virtual bool decode( uint8_t const * data, size_t size, std::vector< uint8_t > & nv12 ) = 0;

        // Returns nullptr when the platform has no H.264 decoder or it failed to initialize
        static std::unique_ptr< h264_decoder_backend > create( int width, int height, int fps );
    };

    // Decodes an H.264 stream into a color format. Unlike the other color converters this block is
    // stateful (frames reference previously decoded ones) and may produce no output for an input
    // frame, in which case nothing is passed downstream.
    class h264_decoder : public color_converter
    {
    public:
        h264_decoder( rs2_format target_format )
            : color_converter( "H264 Decoder", target_format ) {}

        // False when this platform cannot decode H.264; H264 profiles should then stay unexposed
        static bool is_supported();

    protected:
        rs2::frame prepare_output( const rs2::frame_source & source,
                                   rs2::frame input,
                                   std::vector< rs2::frame > results ) override;
        rs2::frame process_frame( const rs2::frame_source & source, const rs2::frame & f ) override;
        void process_function( uint8_t * const dest[], const uint8_t * source, int width, int height,
                               int actual_size, int input_size ) override;

    private:
        std::unique_ptr< h264_decoder_backend > _backend;
        std::vector< uint8_t > _nv12;
        int _width = 0;
        int _height = 0;
    };
}
