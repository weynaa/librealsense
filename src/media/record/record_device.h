// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
#include <core/roi.h>
#include <core/extension.h>
#include <core/serialization.h>
#include "core/streaming.h"
#include "archive.h"
#include "concurrency.h"
#include "sensor.h"
#include "record_sensor.h"

namespace librealsense
{
    class record_device : public device_interface,
                          public extendable_interface,
                          public info_container//TODO: Ziv, does it make sense to inherit from container
    {
    public:
        static const uint64_t MAX_CACHED_DATA_SIZE = 1920 * 1080 * 4 * 30; // ~1 sec of HD video @ 30 FPS

        record_device(std::shared_ptr<device_interface> device, std::shared_ptr<device_serializer::writer> serializer);
        virtual ~record_device();

        std::shared_ptr<context> get_context() const override { return nullptr; } //TODO: Add context

        sensor_interface& get_sensor(size_t i) override;
        size_t get_sensors_count() const override;
        const std::string& get_info(rs2_camera_info info) const override;
        bool supports_info(rs2_camera_info info) const override;
        const sensor_interface& get_sensor(size_t i) const override;
        void hardware_reset() override;


		bool extend_to(rs2_extension extension_type, void** ext) override;
        virtual std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        void pause_recording();
        void resume_recording();
        
    private:
        void write_header();
        std::chrono::nanoseconds get_capture_time() const;
        void write_data(size_t sensor_index, frame_holder f/*, notifications_callback_ptr& sensor_notification_handler*/);
        void write_extension_snapshot(rs2_extension ext, const std::shared_ptr<extension_snapshot>& snapshot);
        std::vector<std::shared_ptr<record_sensor>> create_record_sensors(std::shared_ptr<device_interface> m_device);
        template <typename T> snapshot_collection get_extensions_snapshots(T* extendable);

        std::shared_ptr<device_interface> m_device;
        std::vector<std::shared_ptr<record_sensor>> m_sensors;

        lazy<std::shared_ptr<dispatcher>> m_write_thread;
        std::shared_ptr<device_serializer::writer> m_ros_writer;

        std::chrono::high_resolution_clock::time_point m_capture_time_base;
        std::chrono::high_resolution_clock::duration m_record_pause_time;
        std::chrono::high_resolution_clock::time_point m_time_of_pause;

        std::mutex m_mutex;
        bool m_is_recording;
        std::once_flag m_first_frame_flag;

        uint64_t m_cached_data_size;
        std::once_flag m_first_call_flag;
    };

    MAP_EXTENSION(RS2_EXTENSION_RECORD, record_device);
}

