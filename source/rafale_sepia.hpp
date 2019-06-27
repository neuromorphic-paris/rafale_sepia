#pragma once

#include "../third_party/sepia/source/sepia.hpp"
#include <array>
#include <iostream> // DEBUG
#include <algorithm>
#include <libusb-1.0/libusb.h>
#include <unistd.h>
#include <sstream>

/// rafale_sepia specialises sepia for the Rafale Camera.
/// In order to use this header, an application must link to the dynamic library usb-1.0.
namespace rafale_sepia {

    /// camera represents a rafale (dvs typed) camera.
    class camera {
        public:
        /// available_serials returns the connected cameras serials.
        static std::unordered_set<uint16_t> available_serials() {
            std::cout << "Sensing USB devices" << std::endl;
            std::unordered_set<uint16_t> serials;
            libusb_context* context;
            check_usb_error(libusb_init(&context), "initializing the USB context");
            libusb_device** devices;
            const auto count = libusb_get_device_list(context, &devices);
            for (std::size_t index = 0; index < count; ++index) {
                libusb_device_descriptor descriptor;
                if (libusb_get_device_descriptor(devices[index], &descriptor) == 0) {
                    if (descriptor.idVendor == 1204 && descriptor.idProduct == 244) {
                        libusb_device_handle* handle;
                        check_usb_error(libusb_open(devices[index], &handle), "opening the device");
                        if (libusb_claim_interface(handle, 0) == 0) {
                            auto data = std::array<uint8_t, 8>{};
                            check_usb_error(
                                libusb_control_transfer(handle, 192, 85, 32, 0, data.data(), data.size(), 0),
                                "sending a control packet");
                            libusb_release_interface(handle, 0);
                            serials.insert((static_cast<uint16_t>(data[6]) << 8) | static_cast<uint16_t>(data[7]));
                        }
                        libusb_close(handle);
                    }
                }
            }
            libusb_free_device_list(devices, 1);
            libusb_exit(context);
            return serials;
        }

        struct bias_entry {
            std::string name;
            uint16_t wValue;
            std::array<uint8_t, 4> data;
        };

        // default_parameter returns the default parameter used by the Rafale.
        // For now its a mess, hopefully this will be sorted out and explained at some point. 
        const std::vector<bias_entry> default_biases {
            {"bias_buf", 
                368, {249, 0, 1, 0}},
            {"bias_bulk",
                360, {217, 0, 1, 188}},
            {"bias_cas",
                336, {217, 0, 1, 140}},
            {"bias_clk",
                308, {249, 0, 1, 0}},
            {"bias_del_ack_array",
                292, {217, 0, 1, 140}},
            {"bias_del_reqx_or",
                280, {249, 0, 1, 112}},
            {"bias_del_timeout",
                296, {249, 0, 1, 69}},
            {"bias_diff",
                348, {249, 0, 1, 34}},
            {"bias_diff_off",
                340, {249, 0, 1, 22}},
            {"bias_diff_on",
                344, {249, 0, 1, 45}},
            {"bias_fo",
                352, {217, 0, 1, 240}},
            {"bias_inv",
                300, {249, 0, 1, 62}},
            {"bias_latchout_or_pu",
                264, {217, 0, 1, 183}},
            {"bias_pr",
                356, {217, 0, 1, 197}},
            {"bias_ref",
                304, {217, 0, 1, 211}},
            {"bias_req_pux",
                272, {217, 0, 1, 211}},
            {"bias_req_puy",
                276, {249, 0, 1, 126}},
            {"bias_reqx_or_pu",
                268, {249, 0, 1, 83}},
            {"bias_sendreq_pdx",
                284, {249, 0, 1, 41}},
            {"bias_sendreq_pdy",
                288, {249, 0, 1, 41}},
            {"bias_spare12",
                312, {56, 32, 1, 0}},
            {"bias_spare13",
                316, {56, 32, 1, 0}},
            {"bias_spare14",
                320, {56, 32, 1, 0}},
            {"bias_spare15",
                324, {56, 32, 1, 0}},
            {"bias_spare16",
                328, {56, 32, 1, 0}},
            {"bias_spare17",
                332, {56, 32, 1, 0}},
            {"bias_spare25",
                364, {217, 0, 1, 204}},
            {"bias_cfg_register_high",
                256, {0, 112, 142, 0}},
            {"bias_cfg_register_low",
                260, {0, 2, 232, 0}},
        };

        const std::vector<bias_entry>& get_default_biases() {
            return default_biases;
        }

        std::vector<bias_entry> create_biases(uint8_t sensitivity, uint8_t polarity) {
            const uint8_t _diff_off_index = 8;
            const uint8_t _diff_on_index = 9;

            const uint8_t _max_diff_on = 76;
            const uint8_t _min_diff_on = 39;
            const uint8_t _max_diff_off = 28;
            const uint8_t _min_diff_off = 0;

            auto new_biases = get_default_biases();
            
            auto diff_on_initial = static_cast<uint8_t>(39 + (120-polarity)*(95-sensitivity) * 0.005);
            auto diff_off_initial = static_cast<uint8_t>(28 + (polarity-5)*(sensitivity-90) * 0.007); // Don't ask. idk 

            new_biases[_diff_on_index].data[3] = std::max(std::min(diff_on_initial, _max_diff_on), _min_diff_on);
            new_biases[_diff_off_index].data[3] = std::max(std::min(diff_off_initial, _max_diff_off), _min_diff_off);

            //std::cout << "New biases : sensitivity= " << +sensitivity << ", polarity= " << +polarity << " -> diff_on= " << +new_biases[_diff_on_index].data[3] << ", diff_off= " << +new_biases[_diff_off_index].data[3] << std::endl;

            return new_biases;
        }

        // At some point, other biases are forcefully sent. We put here the forced values, along with some values fitting the default values.
        // They are written here as developpement data, but are not used at this point, the correct behaviour having been found.
        /*
        std::vector<bias_entry> partial_mod_biases {
            {"bias_buf", 
                368, {249, 0, 1, 0}},
            {"bias_bulk",
                360, {217, 0, 1, 255}}, // Byte 3
            {"bias_cas",
                336, {217, 0, 1, 140}},
            {"bias_clk",
                308, {249, 0, 1, 0}},
            {"bias_del_ack_array",
                292, {249, 0, 1, 0}}, // Bytes 0 and 3
            {"bias_del_reqx_or",
                280, {249, 0, 1, 0}}, // Byte 3
            {"bias_del_timeout",
                296, {249, 0, 1, 0}}, // Byte 3
            {"bias_diff",
                348, {249, 0, 1, 5}}, // Byte 3
            {"bias_diff_off",
                340, {249, 0, 1, 0}}, // Byte 3
            {"bias_diff_on",
                344, {249, 0, 1, 13}}, // Byte 3
            {"bias_fo",
                352, {217, 0, 1, 255}}, // Byte 3
            {"bias_inv",
                300, {249, 0, 1, 45}}, 
            {"bias_latchout_or_pu",
                264, {217, 0, 1, 255}}, // Byte 3
            {"bias_pr",
                356, {217, 0, 1, 255}}, // Byte 3
            {"bias_ref",
                304, {217, 0, 1, 140}}, 
            {"bias_req_pux",
                272, {217, 0, 1, 255}}, // Byte 3
            {"bias_req_puy",
                276, {217, 0, 1, 255}}, // Bytes 0 and 3
            {"bias_reqx_or_pu",
                268, {217, 0, 1, 255}}, // Bytes 0 and 3
            {"bias_sendreq_pdx",
                284, {249, 0, 1, 0}}, // Byte 3
            {"bias_sendreq_pdy",
                288, {249, 0, 1, 0}}, // Byte 3
            {"bias_spare12",
                312, {56, 32, 1, 0}},
            {"bias_spare13",
                316, {56, 32, 1, 0}},
            {"bias_spare14",
                320, {56, 32, 1, 0}},
            {"bias_spare15",
                324, {56, 32, 1, 0}},
            {"bias_spare16",
                328, {56, 32, 1, 0}},
            {"bias_spare17",
                332, {56, 32, 1, 0}},
            {"bias_spare25",
                364, {217, 0, 1, 140}},
            {"bias_cfg_register_high",
                256, {0, 112, 142, 0}},
            {"bias_cfg_register_low",
                260, {0, 2, 232, 0}},
        };
        */

        struct command_entry {
            uint16_t wValue;
            std::array<uint8_t, 4> data;
        };


        std::vector<command_entry> biases_intercommands_first {
            {  0, {0, 0, 1,  0}},
            { 26, {0, 0, 0, 17}},
            {  8, {0, 0, 0,  4}},
            {  8, {0, 0, 1,  4}},
            { 10, {0, 0, 0,  1}},
            { 10, {0, 0, 0,  0}}, // Flushing command triggering bulk_flush_method
            {  8, {0, 0, 0,  4}},
            { 10, {0, 0, 0,  1}},
            {  8, {0, 0, 0,  4}},
            {  8, {0, 0, 0, 12}},
            { 22, {0, 0, 4,  0}},
            { 10, {0, 0, 0,  4}},
            { 10, {0, 0, 0, 64}},
            { 10, {0, 0, 0,  1}},
            { 10, {0, 0, 0,128}},
            {  0, {0, 0, 1,  0}},
            { 26, {0, 0, 0, 17}},
            {  0, {0, 0, 1,128}},
            { 10, {0, 0, 0,  1}}
        };
    
        /*
        std::vector<command_entry> biases_intercommands_second {
            { 10, {0, 0, 0, 64}},
            {  0, {0, 1, 1,128}},
            {  0, {0, 3, 1,128}},
            {  0, {0, 7, 1,128}},
            {  0, {0,15, 1,128}},
            {864, {0, 0, 0,  1}},
            {864, {0, 0, 0,  3}},
            {864, {0, 0, 0,  7}},
            {864, {0, 0, 0, 15}},
            {864, {0, 0, 0, 31}}
        };
        */

        std::vector<command_entry> biases_intercommands_third {
            //{ 10, {0, 0, 0, 64}}, // Biases confirmation / end of setting command
            { 10, {0, 0, 0,  4}},
            { 10, {0, 0, 0,128}},
            { 10, {0, 0, 0,  4}},
            { 10, {0, 0, 0,128}},
            { 26, {0, 0, 0, 17}},
            {  8, {0, 0, 1, 12}},
            {  8, {0, 0, 3, 12}},
            {  0, {0,15, 1,128}},
            {  0, {0,15, 1,128}},
            {  0, {0,15, 1,128}},
            {  0, {0,15, 1,128}},
            {  0, {0,15, 1,128}},
            {864, {0, 0, 0, 31}}
        };

        /*
        std::vector<command_entry> trigger_protocol_commands {
            { 10, {0, 0, 0, 64}},
            {768, {0, 0, 0,  0}},
            {768, {0, 0, 0,  0}},
            {832, {0, 0, 0,  0}},
            {832, {0, 0, 0,  0}}
        };
        */

        static void send_biases(libusb_device_handle* _handle, std::vector<bias_entry> biases, const std::string& message) {
            if (message.size() > 0) {
                std::cout << "Sending biases : " << message << std::endl;
            }
            for (const auto& bias : biases){
                send_command(_handle, bias.wValue, bias.data, bias.name);
            }
            command_entry biases_confirmation = {10, {0, 0, 0, 64}};
            send_command(_handle, biases_confirmation.wValue, biases_confirmation.data, "Biases confirmation");
        }

        static void bulk_flush_method(libusb_device_handle* _handle) {
            std::cout << "Flushing 0" << std::endl;
            send_command(_handle, 0xa, {0, 0, 0, 0}, "Flushing 0");
            auto data = std::array<uint8_t, 1024>{};
            int32_t transferred;
            libusb_bulk_transfer(_handle, 129, data.data(), data.size(), &transferred, 100);
        }

        static void send_biases_intercommands(libusb_device_handle* _handle, std::vector<command_entry> commands, const std::string& message) {
            std::cout << "Sending intercommands : " << message << std::endl;
            for (const auto& command : commands) {
                if (command.wValue == 10 && command.data == std::array<uint8_t, 4>{0,0,0,0}) { // if its a flush command
                    bulk_flush_method(_handle);
                }
                else {
                    send_command(_handle, command.wValue, command.data, "Pre Bias Command");
                }
            }
        }

        static void initialize_usb_context_and_search_device(libusb_context** context_ptr, libusb_device_handle** handle_ptr, uint16_t serial_asked, uint64_t MaxStatusTrials) {
            // initialize the context
            check_usb_error(libusb_init(context_ptr), "initializing the USB context");

            // find requested / available devices
            {
                auto device_found = false;
                libusb_device** devices;
                const auto count = libusb_get_device_list(*context_ptr, &devices);
                for (std::size_t index = 0; index < count; ++index) {
                    libusb_device_descriptor descriptor;
                    if (libusb_get_device_descriptor(devices[index], &descriptor) == 0) {
                        if (descriptor.idVendor == 1204 && descriptor.idProduct == 244) {
                            check_usb_error(libusb_open(devices[index], handle_ptr), "opening the device");
                            if (libusb_claim_interface(*handle_ptr, 0) == 0) {
                                device_found = wakeup_method(handle_ptr, serial_asked, 0);
                                
                                if (device_found) {
                                    break;
                                } else {
                                    libusb_release_interface(*handle_ptr, 0);
                                }
                            }
                            libusb_close(*handle_ptr);
                        }
                    }
                }
                libusb_free_device_list(devices, 1);
                if (!device_found) {
                    libusb_exit(*context_ptr);
                    throw sepia::no_device_connected("Rafale DVS");
                }
            }
        }

        // Method wraping up all the pipeline to start talking to the device.
        // Includes 112, 113, 85 (0 & 32)
        static bool wakeup_method(libusb_device_handle** handle_ptr, uint16_t serial_asked, uint64_t MaxStatusTrials) {
            std::cout << "Initiating wakeup procedure." << std::endl;
            std::cout << "Handle is " << *handle_ptr << std::endl;
            auto up_data = std::array<uint8_t, 4>{};
            check_usb_error(
                    libusb_control_transfer(*handle_ptr, 192, 112, 0, 0, up_data.data(), up_data.size(), 0),
                    "sending a control packet");
            if (up_data != std::array<uint8_t, 4>{0,0,2,0}) { return false; }

            bool ready = false;
            uint64_t trials = 0;
            while (!ready && (MaxStatusTrials == 0 || trials < MaxStatusTrials)) {
                auto status_data = std::array<uint8_t, 4>{};
                check_usb_error(
                        libusb_control_transfer(*handle_ptr, 192, 113, 0, 0, status_data.data(), status_data.size(), 0),
                        "sending a control packet");
                ready = (status_data[2] == 1);
            }
            if (!ready) {return false; }

            auto camera_type_data = std::array<uint8_t, 8>{};
            check_usb_error(
                    libusb_control_transfer(*handle_ptr, 192, 85, 0, 0, camera_type_data.data(), camera_type_data.size(), 0),
                    "sending a control packet");
            //if (camera_type_data != std::array<uint8_t, 8>{64, 0, 0, 0, 0, 3, 32, 160}) { return false; }

            auto camera_id_data = std::array<uint8_t, 8>{};
            check_usb_error(
                    libusb_control_transfer(*handle_ptr, 192, 85, 32, 0, camera_id_data.data(), camera_id_data.size(), 0),
                    "sending a control packet");
            if (serial_asked != 0 && ((serial_asked & 0xff) != camera_id_data[7] || ((serial_asked & 0xff00) >> 8) != camera_id_data[6])) { return false; }

            return true;
        }

        /// width returns the sensor width.
        static constexpr uint16_t width() {
            return 640;
        }

        /// height returns the sensor height.
        static constexpr uint16_t height() {
            return 480;
        }

        camera() = default;
        camera(const camera&) = delete;
        camera(camera&&) = default;
        camera& operator=(const camera&) = delete;
        camera& operator=(camera&&) = default;
        virtual ~camera() {}

        protected:
        /// check_usb_error throws if the given value is not zero.
        static void check_usb_error(int error, const std::string& message) {
            if (error < 0) {
                throw std::logic_error(message + " failed: " + libusb_strerror(static_cast<libusb_error>(error)));
            }
        }

        /// send_command sends a setup command to the given camera.
        static void send_command(
            libusb_device_handle* handle,
            uint16_t w_value,
            std::array<uint8_t, 4> data,
            const std::string& message) {
            check_usb_error(libusb_control_transfer(handle, 64, 86, w_value, 0, data.data(), data.size(), 0), message);
        }
    };

    /// specialized_camera represents a template-specialized Rafale.
    template <typename HandleEvent, typename HandleException>
    class specialized_camera : public camera,
                               public sepia::specialized_camera<sepia::dvs_event, HandleEvent, HandleException> {
        public:
        specialized_camera<HandleEvent, HandleException>(
            HandleEvent handle_event,
            HandleException handle_exception,
            std::size_t fifo_size,
            uint16_t serial,
            std::chrono::milliseconds sleep_duration) :
            sepia::specialized_camera<sepia::dvs_event, HandleEvent, HandleException>(
                std::forward<HandleEvent>(handle_event),
                std::forward<HandleException>(handle_exception),
                fifo_size,
                sleep_duration),
            _acquisition_running(true) {

            initialize_usb_context_and_search_device(&_context, &_handle, serial, 0);
            bulk_flush_method(_handle);

            if (true) {
                std::cout << "rafale_verbose : #35" << std::endl;
                auto data = std::array<uint8_t, 8>{};
                check_usb_error(
                    libusb_control_transfer(_handle, 192, 85, 2052, 0, data.data(), data.size(), 0),
                    "sending a control packet");
            }
            
            {
                auto data = std::array<uint8_t, 4>{};
                check_usb_error(
                        libusb_control_transfer(_handle, 192, 114, 0, 0, data.data(), data.size(), 0),
                        "sending a control packet");
            }
            wakeup_method(&_handle, serial, 0);


            send_biases_intercommands(_handle, biases_intercommands_first, "first");
            send_biases(_handle, default_biases, "default");
            send_biases_intercommands(_handle, biases_intercommands_third, "third");


            // start the reading loop
            _acquisition_loop = std::thread([this, serial]() -> void {
                try {
                    auto data = std::vector<uint8_t>(1 << 17);
                    sepia::dvs_event event;
                    uint64_t t_offset = 1;
                    uint64_t last_static_offset = 1;
                    uint64_t last_ts = 0;

                    const uint64_t max_ts_delta = 1e5;
                    const uint64_t max_ts_trigger = 1e5;

                    const bool _auto_update_biases = true;
                    const uint8_t _max_sensitivity = 70;
                    const uint8_t _min_sensitivity = 8;
                    const uint8_t _max_polarity = 70;
                    const uint8_t _min_polarity = 16;
                    const uint8_t _sensitivity_mod = 4;
                    const uint8_t _polarity_mod = 4;
                    const uint64_t _biases_autoupdate_delta_ts = 5*1e5; // in µs

                    const uint64_t _event_rate_output = 1e6;
                    
                    const uint64_t _evs_max_cap = 1*1e6; // in ev/s
                    const uint64_t _evs_min_cap = 1e3; // in ev/s
                    const uint8_t _pol_ratio_cap = 8; // Changes polarity if OFF events are _pol_ratio_cap more numerous than ON events, or inverted

                    uint8_t sensitivity = 30;
                    uint8_t polarity = 50;
                    uint64_t last_update = 0;
                    uint64_t last_output = 0;
                    
                    uint64_t on_events = 0;
                    uint64_t off_events = 0;

                    bool initialized = false;

                    std::cout << "Starting polling" << std::endl;
                    while (_acquisition_running.load(std::memory_order_relaxed)) {
                        int32_t transferred = 0;
                        const auto error = libusb_bulk_transfer(
                            _handle,
                            129,
                            data.data(),
                            static_cast<uint32_t>(data.size()),
                            &transferred,
                            static_cast<uint32_t>(this->_sleep_duration.count()));
                        if ((error == 0 || error == LIBUSB_ERROR_TIMEOUT) && transferred % 4 == 0) {
                            for (auto byte_iterator = data.begin();
                                 byte_iterator != std::next(data.begin(), transferred);
                                 std::advance(byte_iterator, 4)) {
                                if (*std::next(byte_iterator, 3) == 128) {
                                    uint64_t t_offset_received = (static_cast<uint64_t>(*byte_iterator)
                                                | (static_cast<uint64_t>(*std::next(byte_iterator, 1)) << 8)    // Current t_offset computation only deals with bytes 0, 1 and 2, thus reaching a maximum of ~1073s.
                                                | (static_cast<uint64_t>(*std::next(byte_iterator, 2)) << 16))  // Some bytes remain on byte 3, possibly increasing up to ~120h (using all 5 bits : 0b11111). 
                                               * 0x40;                                                          // Yet no test has reached this timestamp, so this behavious is only a possibility. Thus not implemented.
                                    if (t_offset_received == last_static_offset) {
                                        t_offset += 16;
                                    } else {
                                        last_static_offset = t_offset_received;
                                        t_offset = last_static_offset;
                                        //std::stringstream Stream;
                                        //std::cout << Stream.str() << std::flush;
                                    }
                                } 
                                else if (*std::next(byte_iterator, 3) == 160) {}
                                else if (*byte_iterator == 255 && *std::next(byte_iterator, 1) == 255 && *std::next(byte_iterator, 2) == 255) {}
                                else {
                                    event.y = (height()-1) - ((static_cast<uint16_t>(*std::next(byte_iterator, 1) & 0b1) << 8)
                                              | (*byte_iterator)) ;
                                    if (!(event.y < height())) {
                                        continue;
                                    }
                                    event.t = t_offset
                                              + ((static_cast<uint64_t>((*std::next(byte_iterator, 3) & 0b11)) << 2)
                                                 | (*std::next(byte_iterator, 2) >> 6));
                                    if (!initialized) {
                                        if (event.t < max_ts_trigger) {
                                            initialized = true;
                                            last_ts = event.t;
                                            //std::cout << "last_ts initialized : " << last_ts << std::endl;
                                        } else {
                                            continue;
                                        }
                                    }
                                    if (event.t < last_ts || event.t > last_ts + max_ts_delta) {
                                        continue;
                                    }
                                    event.x = (static_cast<uint16_t>(*std::next(byte_iterator, 2) & 0b11111) << 5)
                                              | (*std::next(byte_iterator, 1) >> 3);
                                    event.is_increase = ((*std::next(byte_iterator, 3) >> 4) & 0b1) == 1;
                                    if (event.is_increase) {
                                        on_events++;
                                    } else {
                                        off_events++;
                                    }

                                    const auto delta_ts = event.t - last_update;
                                    if (delta_ts >= _biases_autoupdate_delta_ts) {
                                        const auto sum_events = on_events + off_events;
                                        const auto previous_sensitivity = sensitivity;
                                        const auto previous_polarity = polarity;
                                        if (sum_events * 1e6 > _evs_max_cap * delta_ts) { // We had in here too many events
                                            sensitivity = (sensitivity <= _min_sensitivity ? _min_sensitivity : sensitivity - _sensitivity_mod);
                                        } else if (sum_events * 1e6 < _evs_min_cap * delta_ts) { // We had in here too few events
                                            sensitivity = (sensitivity >= _max_sensitivity ? _max_sensitivity : sensitivity + _sensitivity_mod);
                                        }
                                        if (on_events > _pol_ratio_cap * off_events) {
                                            polarity = (polarity <= _min_polarity ? _min_polarity : polarity - _polarity_mod);
                                        } else if (off_events > _pol_ratio_cap * on_events) {
                                            polarity = (polarity >= _max_polarity ? _max_polarity : polarity + _polarity_mod);
                                        }
                                        last_update = event.t;
                                        if (event.t - last_output > _event_rate_output) {
                                            std::cout << "Event rate : " << sum_events * 1e6 / delta_ts << ", on vs off is " << 100.0* on_events / sum_events << " (s=" << +sensitivity << ", p=" << +polarity << ")" << std::endl;
                                            last_output = event.t;
                                        }
                                        on_events = 0;
                                        off_events = 0;

                                        if (polarity != previous_polarity || sensitivity != previous_sensitivity) {
                                            if (_auto_update_biases) {
                                                auto updated_biases = create_biases(sensitivity, polarity);
                                                send_biases(_handle, updated_biases, "");
                                            }
                                        }

                                    }

                                    if (event.x >= width() || event.y >= height()) {
                                        std::cout << "Data : " << +*byte_iterator << " " << +*std::next(byte_iterator, 1) << " " << +*std::next(byte_iterator, 2) << " " << +*std::next(byte_iterator, 3) << ", event (x, y, t, p) : (" << event.x << ", " << event.y << ", " << event.t << ", " << event.is_increase << ")" << std::endl;
                                    } else {
                                        last_ts = event.t;
                                        if (!this->push(event)) {
                                            throw std::runtime_error("computer's FIFO overflow");
                                        }
                                    }
                                }
                            }
                        } else {
                            throw sepia::device_disconnected("Rafale DVS");
                        }
                    }
                    std::cout << "End of acquisition." << std::endl;
                } catch (...) {
                    this->_handle_exception(std::current_exception());
                }
            });
        }
        specialized_camera(const specialized_camera&) = delete;
        specialized_camera(specialized_camera&&) = default;
        specialized_camera& operator=(const specialized_camera&) = delete;
        specialized_camera& operator=(specialized_camera&&) = default;
        virtual ~specialized_camera() {
            _acquisition_running.store(false, std::memory_order_relaxed);
            _acquisition_loop.join();
            std::cout << "Releasing interface" << std::endl;
            libusb_release_interface(_handle, 0);
            std::cout << "Closing interface" << std::endl;
            libusb_close(_handle);
            std::cout << "Closing USB context" << std::endl;
            libusb_exit(_context);
        }

        protected:
        std::atomic_bool _acquisition_running;
        libusb_context* _context;
        libusb_device_handle* _handle;
        std::thread _acquisition_loop;
    };

    /// make_camera creates a camera from functors.
    template <typename HandleEvent, typename HandleException>
    std::unique_ptr<specialized_camera<HandleEvent, HandleException>> make_camera(
        HandleEvent handle_event,
        HandleException handle_exception,
        std::size_t fifo_size = 1 << 24,
        uint16_t serial = 0x66,
        std::chrono::milliseconds sleep_duration = std::chrono::milliseconds(10)) {
        return sepia::make_unique<specialized_camera<HandleEvent, HandleException>>(
            std::forward<HandleEvent>(handle_event),
            std::forward<HandleException>(handle_exception),
            fifo_size,
            serial,
            sleep_duration);
    }
}
