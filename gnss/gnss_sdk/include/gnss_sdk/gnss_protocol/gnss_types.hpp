#ifndef GNSS_STATE_HPP
#define GNSS_STATE_HPP

#include <cstdint>
#include <iostream>
#include "gnss_sdk/gnss_protocol/gnss_protocol.h"

namespace wescore
{
struct GnssState
{
    char gpgga_data[PAYLOAD_BUFFER_SIZE];
    char gprmc_data[PAYLOAD_BUFFER_SIZE];
    char gpfpd_data[PAYLOAD_BUFFER_SIZE];
    char gtimu_data[PAYLOAD_BUFFER_SIZE];

    int gpgga_get = 0;
    int gprmc_get = 0;
    int gpfpd_get = 0;
    int gtimu_get = 0;
    int loop_len = 10000;
};
} // namespace wescore

#endif /* GNSS_STATE_HPP */
