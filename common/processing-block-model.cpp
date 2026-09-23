// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2024 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <string>
#include "subdevice-model.h"
#include "processing-block-model.h"
#include "control-section.h"
#include "viewer.h"

#include <rsutils/easylogging/easyloggingpp.h>


namespace rs2
{
    processing_block_model::processing_block_model(
        subdevice_model* owner,
        const std::string& name,
        std::shared_ptr<rs2::filter> block,
        std::function<rs2::frame(rs2::frame)> invoker,
        std::string& error_message, bool enable)
        : _owner(owner), _name(name), _block(block), _invoker(invoker), _enabled(enable)
    {
        std::stringstream ss;
        ss << "##" << ((owner) ? owner->dev.get_info(RS2_CAMERA_INFO_NAME) : _name)
            << "/" << ((owner) ? (*owner->s).get_info(RS2_CAMERA_INFO_NAME) : "_")
            << "/" << (long long)this;

        if (_owner)
            _full_name = get_post_processing_device_sensor_name(_owner) + "." + _name;
        else
            _full_name = _name;

        _enabled = restore_processing_block(_full_name.c_str(),
            block, _enabled);

        populate_options(ss.str().c_str(), owner, owner ? &owner->_options_invalidated : nullptr, error_message);
    }

    option_model * processing_block_model::get_option_model( rs2_option opt )
    {
        auto it = _options_id_to_model.find( opt );
        return it == _options_id_to_model.end() ? nullptr : &it->second;
    }

    void processing_block_model::save_to_config_file()
    {
        save_processing_block_to_config_file(_full_name.c_str(), _block, _enabled);
    }

    void processing_block_model::add_options_to( control_section & section, viewer_model & viewer )
    {
        for( auto & id_model : _options_id_to_model )
        {
            if( viewer.is_option_skipped( id_model.first ) )
                continue;

            // written behind the panel's back - by the block itself and by the depth-visualization
            // controls - so these are re-read on every frame they draw
            section.add( std::make_unique< option_control >( id_model.second,
                val_in_range( id_model.first, { RS2_OPTION_MIN_DISTANCE, RS2_OPTION_MAX_DISTANCE,
                                                RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED } ) ) );
        }
    }


    void processing_block_model::populate_options(const std::string& opt_base_label,
        subdevice_model* model,
        bool* options_invalidated,
        std::string& error_message)
    {
        for( option_value option : _block->get_supported_option_values() )
        {
            option_model om = create_option_model( option,
                                                   opt_base_label,
                                                   model,
                                                   _block,
                                                   model ? &model->_options_invalidated : nullptr,
                                                   error_message );
            // Software filter: write synchronously (no FW round-trip) so the value is read back
            // and the control doesn't revert to a stale value.
            om.write_synchronously = true;
            _options_id_to_model[option->id] = om;
        }
    }

    bool restore_processing_block(const char* name,
        std::shared_ptr<rs2::processing_block> pb, bool enable)
    {
        for (auto opt : pb->get_supported_option_values())
        {
            std::string key = name;
            key += ".";
            key += pb->get_option_name(opt->id);
            if (config_file::instance().contains(key.c_str()))
            {
                float val = config_file::instance().get(key.c_str());
                try
                {
                    auto range = pb->get_option_range(opt->id);
                    if (val >= range.min && val <= range.max)
                        pb->set_option(opt->id, val);
                }
                catch (...)
                {
                }
            }
        }

        std::string key = name;
        key += ".enabled";
        if (config_file::instance().contains(key.c_str()))
        {
            return config_file::instance().get(key.c_str());
        }
        return enable;
    }

    void processing_block_model::set_secondary_block( std::shared_ptr< rs2::filter > block )
    {
        _secondary_block = std::move( block );
        _mirrored_options.clear();
        if( ! _secondary_block )
            return;

        for( auto opt : _block->get_supported_options() )
            if( ! _block->is_option_read_only( opt ) )
                _mirrored_options.push_back( opt );
    }

    // The UI writes options to the primary block only, and a block can decide should_process() from an
    // option - stream_filter_processing_block does - so the values have to be carried across before the
    // secondary one runs. Compared first: a redundant set_option resets a filter's internal state.
    rs2::frame processing_block_model::invoke_secondary( rs2::frame f ) const
    {
        for( auto opt : _mirrored_options )
        {
            try
            {
                auto value = _block->get_option( opt );
                if( _secondary_block->get_option( opt ) != value )
                    _secondary_block->set_option( opt, value );
            }
            catch( const rs2::error & e )
            {
                LOG_WARNING( "Could not mirror " << _name << " option " << opt << ": " << e.what() );
            }
        }
        return _secondary_block->process( f );
    }

    void save_processing_block_to_config_file(const char* name,
        std::shared_ptr<rs2::processing_block> pb, bool enable)
    {
        for (auto opt : pb->get_supported_options())
        {
            auto val = pb->get_option(opt);
            std::string key = name;
            key += ".";
            key += pb->get_option_name(opt);
            config_file::instance().set(key.c_str(), val);
        }

        std::string key = name;
        key += ".enabled";
        config_file::instance().set(key.c_str(), enable);
    }
}
