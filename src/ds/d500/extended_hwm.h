// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

#pragma once

#include "hw-monitor.h"

namespace librealsense
{
    // The aim of this class is to permit to send and receive buffers that are bigger than 1KB via the hw monitor mechanism
    // A protocol has been defined using an additional parameter in each message, 
    // that will indicate which chunk of the whole message is currently sent/received 
    // (see following private method compute_chunks_param)
    class extended_hwm_interface
    {
    public:
        enum class buffer_type
        {
            standard,
            extended_receive,
            extended_send
        };

        // Returns needed action for a command - standard HWM operation, extended receive or extended send.
        static buffer_type get_buffer_type( uint32_t opcode, uint32_t param4 );
        static buffer_type get_buffer_type( const std::vector< uint8_t > & data );

        // Returns number of chunks a message will be divided into based on the message length
        static uint16_t get_number_of_chunks( size_t msg_length );
        
        // Gets the data for current extended send iteration out of a data buffer
        static std::vector< uint8_t > get_data_for_current_iteration( const std::vector< uint8_t > & data, int iteration );

        // Extended operations use param4 of the HWM to pass current and max chunck number. Returns the appropriate param4 value for current iteration.
        static uint32_t compute_chunks_param( uint16_t overall_chunks, int iteration );

        // Helper function to build command struct from data vector.
        static command build_command_from_data( const std::vector< uint8_t > & data );
        static std::vector< uint8_t > build_data_from_command( const command & cmd );

    protected:
        // Implementation of actual extended operation
        virtual void send_extended_hwm( const command & cmd ) const = 0;
        virtual std::vector< uint8_t > receive_extended_hwm( const command & cmd ) const = 0;
    };

    class extended_hw_monitor : public extended_hwm_interface, public hw_monitor
    {
    public:
        explicit extended_hw_monitor( std::shared_ptr< locked_transfer > locked_transfer,
                                      std::shared_ptr< hwmon_response_interface > hwmon_response )
            : hw_monitor( locked_transfer, hwmon_response )
        {
        }

        // extended_hwm_interface functionallity
        virtual void send_extended_hwm( const command & cmd ) const override;
        virtual std::vector< uint8_t > receive_extended_hwm( const command & cmd ) const override;

        // hw_monitor functionallity
        virtual std::vector< uint8_t > send( std::vector< uint8_t > const & data ) const override;
        virtual std::vector< uint8_t > send( command const & cmd, hwmon_response_type * = nullptr, bool locked_transfer = false ) const override;

    private:
        void extended_send( const command & cmd, hwmon_response_type * p_response, bool locked_transfer ) const;
        std::vector< uint8_t > extended_receive( const command & cmd, hwmon_response_type * p_response, bool locked_transfer ) const;
    };
}
