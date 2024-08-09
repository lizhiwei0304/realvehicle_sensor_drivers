#include "gnss_sdk/gnss_protocol/gnss_base.hpp"

#include <string>
#include <string.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

namespace
{
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace wescore
{
GnssBase::~GnssBase()
{
    if (serial_connected_)
        serial_if_->close();
    std::cout<<"close"<<std::endl;
}

void GnssBase::Connect(std::string dev_name, int32_t baud_rate)
{
    if (baud_rate != 0)
    {
        ConfigureSerial(dev_name, baud_rate);

        if (!serial_connected_)
            std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
    }
}

void GnssBase::Disconnect()
{
    if (serial_connected_)
    {
        if (serial_if_->is_open())
            serial_if_->close();
    }
}


void GnssBase::ConfigureSerial(const std::string uart_name, int32_t baud_rate)
{
    serial_if_ = std::make_shared<ASyncSerial>(uart_name, baud_rate);
    serial_if_->open();

    if (serial_if_->is_open())
        serial_connected_ = true;

    serial_if_->set_receive_callback(std::bind(&GnssBase::ParseUARTBuffer, this,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3));
}

GnssState GnssBase::GetGnssState()
{
    std::lock_guard<std::mutex> guard(gnss_state_mutex_);
    return gnss_state_;
}

void GnssBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    GnssStatusMessage status_msg;
    for (int i = 0; i < bytes_received; ++i)
    {
        if (DecodeGnssStatusMsgFromUART(buf[i], &status_msg))
            NewStatusMsgReceivedCallback(status_msg);
    }
}

void GnssBase::NewStatusMsgReceivedCallback(const GnssStatusMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(gnss_state_mutex_);
    UpdateGnssState(msg, gnss_state_);
}

void GnssBase::UpdateGnssState(const GnssStatusMessage &status_msg, GnssState &state)
{
    switch (status_msg.msg_type)
    {
    case GnssGPGGAMsg:
    {
        state.gpgga_get = (state.gpgga_get+1) % state.loop_len;
        strcpy(state.gpgga_data, status_msg.gnss_status_msg);
        break;
    }
    case GnssGPRMCMsg:
    {
        state.gprmc_get = (state.gprmc_get+1) % state.loop_len;
        strcpy(state.gprmc_data, status_msg.gnss_status_msg);
        break;
    }
    case GnssGPFPDMsg:
    {
        state.gpfpd_get = (state.gpfpd_get+1) % state.loop_len;
        strcpy(state.gpfpd_data, status_msg.gnss_status_msg);
        break;
    }
    case GnssGTIMUMsg:
    {
        state.gtimu_get = (state.gtimu_get+1) % state.loop_len;
        strcpy(state.gtimu_data, status_msg.gnss_status_msg);
        break;
    }
    default:
        break;
    }
}
} // namespace wescore
