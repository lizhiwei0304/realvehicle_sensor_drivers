#ifndef GNSS_PROTOCOL_H
#define GNSS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*-------------------- Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Note: id could be different for UART and CAN protocol

// System Status Feedback

#define PAYLOAD_BUFFER_SIZE 150

typedef enum
{
    GnssStatusNone = 0x00,
    GnssGPGGAMsg = 0x01,
    GnssGPRMCMsg = 0x02,
    GnssGPFPDMsg = 0x03,
    GnssGTIMUMsg = 0x04
} GnssStatusMsgType;

typedef struct {
    GnssStatusMsgType msg_type;
    char gnss_status_msg[PAYLOAD_BUFFER_SIZE];
} GnssStatusMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* GNSS_PROTOCOL_H */
