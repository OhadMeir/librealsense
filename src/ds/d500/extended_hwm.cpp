// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

#include "extended_hwm.h"
#include <ds/ds-private.h>


namespace librealsense
{
    extended_hwm_interface::buffer_type extended_hwm_interface::get_buffer_type( uint32_t opcode, uint32_t param4 )
    {
        bool provide_whole_table = ( param4 == 0 );
        switch( opcode )
        {
        case ds::fw_cmd::GET_HKR_CONFIG_TABLE:
            return provide_whole_table ? buffer_type::extended_receive : buffer_type::standard;
        case ds::fw_cmd::SET_HKR_CONFIG_TABLE:
            return provide_whole_table ? buffer_type::extended_send : buffer_type::standard;
        default:
            return buffer_type::standard;
        }
    }

    extended_hwm_interface::buffer_type extended_hwm_interface::get_buffer_type( const std::vector< uint8_t > & data )
    {
        uint32_t opcode_offset = 4;  // Skipping first 4 bytes - data size + magic number
        uint8_t opcode = data[opcode_offset];

        uint32_t param4_offset = 16;  // Skipping also opcode + param1 + param2 + param3
        uint32_t param4 = data[param4_offset];

        return get_buffer_type( opcode, param4 );
    }

    uint16_t extended_hwm_interface::get_number_of_chunks( size_t msg_length )
    {
        return static_cast< uint16_t >( std::ceil( msg_length / static_cast< float >( HW_MONITOR_COMMAND_SIZE ) ) );
    }

    std::vector< uint8_t > extended_hwm_interface::get_data_for_current_iteration( const std::vector< uint8_t > & data, int iteration )
    {
        auto remaining_data_size_to_be_sent = data.size() - iteration * HW_MONITOR_COMMAND_SIZE;

        auto start_iterator_for_current_chunk = data.begin() + iteration * HW_MONITOR_COMMAND_SIZE;
        auto end_iterator_for_current_chunk = ( remaining_data_size_to_be_sent >= HW_MONITOR_COMMAND_SIZE )
                                                ? start_iterator_for_current_chunk + HW_MONITOR_COMMAND_SIZE
                                                : start_iterator_for_current_chunk + remaining_data_size_to_be_sent;

        return std::vector< uint8_t >( start_iterator_for_current_chunk, end_iterator_for_current_chunk );
    }

    // The following method prepares the param4 of the command for extended buffers
    // The aim of this param is to send the chunk number out of max of expected chunks for the current command
    // It results to a 32 bit integer, which is built from two 16 bits ints:
    // Chunk Id / Max Chunk id
    // uint16(LSB / Little Endian) - current chunk id(0...N - 1)
    // uint16(MSB / Little Endian) - max chunk id(N - 1)
    // Each table is split into N chunks of 1000 bytes as per XU limit
    // The last chunk is within 1..1000 range by design
    // examples:
    // Sending chunk 1 out of 1 chunk => current chunk = 0, expected chunks = 0 (means 1), so param = 0
    // Sending chunk 2 out of 3 chunks => current chunk = 1, expected chunks = 2 (means 3), so param = 0x00020001
    uint32_t extended_hwm_interface::compute_chunks_param( uint16_t overall_chunks, int iteration )
    {
        return ( ( overall_chunks - 1 ) << 16 ) | iteration;
    }


    command extended_hwm_interface::build_command_from_data( const std::vector< uint8_t > & data )
    {
        uint32_t offset = 4;  // Skipping over first 4 bytes - data size + magic number
        uint8_t opcode = data[offset];
        offset += sizeof( uint32_t ); // Despite the fact that the opcode is only uint8, we need to step over 4 bytes,
        uint32_t param1 = *reinterpret_cast< const uint32_t * >( data.data() + offset );
        offset += sizeof( uint32_t );
        uint32_t param2 = *reinterpret_cast< const uint32_t * >( data.data() + offset );
        offset += sizeof( uint32_t );
        uint32_t param3 = *reinterpret_cast< const uint32_t * >( data.data() + offset );
        offset += sizeof( uint32_t );
        uint32_t param4 = *reinterpret_cast< const uint32_t * >( data.data() + offset );
        offset += sizeof( uint32_t );

        command cmd{ opcode, param1, param2, param3, param4 };
        if( data.size() > offset )
            cmd.data.insert( cmd.data.begin(), data.begin() + offset, data.end() );

        return cmd;
    }

    std::vector< uint8_t > extended_hwm_interface::build_data_from_command( const command & cmd )
    {
        constexpr const uint32_t header_size = 24;  
        std::vector< uint8_t > data( cmd.data.size() + header_size, 0 );
        uint8_t * index = data.data();

        uint16_t tmp_size = static_cast< uint16_t >( cmd.data.size() + header_size - 4 ); // Without size + magic number fields
        memcpy( index, &tmp_size, sizeof( tmp_size ) );
        index += sizeof( uint16_t );
        uint16_t magic_number = 0xcdab;
        memcpy( index, &magic_number, sizeof( magic_number ) );
        index += sizeof( uint16_t );
        memcpy( index, &cmd.cmd, sizeof( cmd.cmd ) );
        index += sizeof( uint32_t ); // Despite the fact that the opcode is only uint8, we need to step over 4 bytes,
        memcpy( index, &cmd.param1, sizeof( cmd.param1 ) );
        index += sizeof( cmd.param1 );
        memcpy( index, &cmd.param2, sizeof( cmd.param2 ) );
        index += sizeof( cmd.param2 );
        memcpy( index, &cmd.param3, sizeof( cmd.param3 ) );
        index += sizeof( cmd.param3 );
        memcpy( index, &cmd.param4, sizeof( cmd.param4 ) );
        index += sizeof( cmd.param4 );
        memcpy( index, cmd.data.data(), cmd.data.size() );

        return data;
    }

    void extended_hw_monitor::send_extended_hwm( const command & cmd ) const
    {
        extended_send( cmd, nullptr, false );
    }

    std::vector< uint8_t > extended_hw_monitor::receive_extended_hwm( const command & cmd ) const
    {
        return extended_receive( cmd, nullptr, false );
    }

    std::vector< uint8_t > extended_hw_monitor::send( std::vector< uint8_t > const & data ) const
    {
        extended_hwm_interface::buffer_type buffer_type = get_buffer_type( data );
        if( buffer_type == extended_hwm_interface::buffer_type::standard )
            return hw_monitor::send( data );

        // returning the hwmc answer with 4 bytes for opcode as header
        // this is needed because the hw_monitor::send with command is used, while
        // the hw_monitor::send with vector<uint8_t> has been used
        command cmd = build_command_from_data( data );
        std::vector< uint8_t > ans_from_hwmc = send( cmd );
        std::vector< uint8_t > rv;
        uint32_t opcode_data = static_cast< uint32_t >( cmd.cmd );
        rv.insert( rv.end(),
                   reinterpret_cast< uint8_t * >( &opcode_data ),
                   reinterpret_cast< uint8_t * >( &opcode_data ) + sizeof( opcode_data ) );
        rv.insert( rv.end(), ans_from_hwmc.begin(), ans_from_hwmc.end() );
        return rv;
    }

    // 3 cases are foreseen in this method:
    // - no buffer bigger than 1 KB is expected with the current command => work as normal hw_monitor::send method
    // - buffer bigger than 1 KB expected to be received => iterate the hw_monitor::send method and append the results
    // - buffer bigger than 1 KB expected to be sent => iterate the hw_monitor, while iterating over the input
    std::vector<uint8_t> extended_hw_monitor::send(command const & cmd, hwmon_response_type* p_response, bool locked_transfer) const
    {
        extended_hwm_interface::buffer_type buffer_type = extended_hwm_interface::get_buffer_type( cmd.cmd, cmd.param4 );
        switch( buffer_type)
        {
        case extended_hwm_interface::buffer_type::standard:
            return hw_monitor::send(cmd, p_response, locked_transfer);
        case extended_hwm_interface::buffer_type::extended_receive:
            return extended_receive(cmd, p_response, locked_transfer);
        case extended_hwm_interface::buffer_type::extended_send:
            extended_send(cmd, p_response, locked_transfer);
            break;
        }

        return std::vector<uint8_t>();
    }

    void extended_hw_monitor::extended_send( const command & cmd, hwmon_response_type * p_response, bool locked_transfer ) const
    {
        command curr_chunk_cmd( cmd.cmd, cmd.param1, cmd.param2, cmd.param3 );
        uint16_t overall_chunks = extended_hwm_interface::get_number_of_chunks( cmd.data.size() );

        for( int i = 0; i < overall_chunks; ++i )
        {
            curr_chunk_cmd.param4 = extended_hwm_interface::compute_chunks_param( overall_chunks, i ); // Chunk number is in param4
            curr_chunk_cmd.data = extended_hwm_interface::get_data_for_current_iteration( cmd.data, i );

            hw_monitor::send( curr_chunk_cmd, p_response, locked_transfer );
        }
    }

    std::vector< uint8_t > extended_hw_monitor::extended_receive( const command & cmd,
                                                                  hwmon_response_type * p_response,
                                                                  bool locked_transfer ) const
    {
        std::vector< uint8_t > recv_msg;

        // send first command with 0/0 on param4, this should get the first chunk withoud knowing
        // the actual table size, actual size will be returned as part for the response header and
        // will be used to calculate the extended loop range
        auto ans = hw_monitor::send( cmd, p_response, locked_transfer );
        if( p_response && *p_response != cmd.cmd ) // hw_monitor::send will throw if p_response is nullptr and an error occured
            return ans;
        recv_msg.insert( recv_msg.end(), ans.begin(), ans.end() );

        if( recv_msg.size() < sizeof( ds::table_header ) )
            throw std::runtime_error( rsutils::string::from() << "Table data has invalid size = " << recv_msg.size() );

        ds::table_header * th = reinterpret_cast< ds::table_header * >( ans.data() );
        size_t recv_msg_length = sizeof( ds::table_header ) + th->table_size;

        if( recv_msg_length > HW_MONITOR_BUFFER_SIZE )
        {
            command curr_chunk_cmd( cmd.cmd, cmd.param1, cmd.param2, cmd.param3 );
            uint16_t overall_chunks = extended_hwm_interface::get_number_of_chunks( recv_msg_length );

            // Since we already have the first chunk we start the loop from index 1
            for( int i = 1; i < overall_chunks; ++i )
            {
                curr_chunk_cmd.param4 = extended_hwm_interface::compute_chunks_param( overall_chunks, i ); // Chunk number is in param4

                ans = hw_monitor::send( curr_chunk_cmd, p_response, locked_transfer );
                if( p_response && *p_response != cmd.cmd ) // hw_monitor::send will throw if p_response is nullptr and an error occured
                    return ans;
                recv_msg.insert( recv_msg.end(), ans.begin(), ans.end() );
            }
        }

        return recv_msg;
    }
    }
