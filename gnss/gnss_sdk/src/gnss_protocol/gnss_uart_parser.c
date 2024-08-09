#include "gnss_sdk/gnss_protocol/gnss_uart_parser.h"

//$GPGGA,000000.00,0000.0000,S,00000.0000,W,0,00,0.0,0.00,M,0.00,M,,*00
typedef enum
{
    WAIT_FOR_HEAD = 0,
    WAIT_FOR_DATA,
} GnssSerialDecodeState;

typedef union {
    GnssStatusMessage status_msg;
} GnssDecodedMessage;

// frame buffer
static char data[PAYLOAD_BUFFER_SIZE];
static uint8_t data_pos = 0;
static uint8_t frame_checksum = 0;
static uint8_t checksum_buffer[2];
static uint8_t checksum_data_pos = 0;
static bool checksum_flag = false;
static uint8_t internal_checksum = 0;
static bool ParseChar(uint8_t c, GnssDecodedMessage *msg);
static uint8_t Ascii2Hex(uint8_t ch);
static uint8_t CalcGPGGAChecksum(char str[], uint8_t len);
static bool ConstructStatusMessage(GnssStatusMessage *msg);

bool DecodeGnssStatusMsgFromUART(uint8_t c, GnssStatusMessage *msg)
{
    static GnssDecodedMessage decoded_msg;

    bool result = ParseChar(c, &decoded_msg);
    if (result)
        *msg = decoded_msg.status_msg;
    return result;
}

uint8_t CalcUARTChecksum(uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;

    for (int i = 0; i < len; ++i)
        checksum += buf[i];

    return checksum;
}

uint8_t Ascii2Hex(uint8_t ch)
{
    if (ch >= '0' && ch <= '9')
        ch = ch - '0' + 0x00;
    else if (ch >= 'A' && ch <= 'F')
        ch = ch - 'A' + 0x0A;
    return ch;
}

uint8_t CalcGPGGAChecksum(char str[], uint8_t len)
{
     uint8_t x = (uint8_t)str[1];
     uint8_t y;
     for(size_t i=2;i<len;i++)
     {
        y=(uint8_t)str[i];
        x=x^y;
     }

     return x;
}

bool ParseChar(uint8_t c, GnssDecodedMessage *msg)
{
    static GnssSerialDecodeState decode_state = WAIT_FOR_HEAD;

    bool new_frame_parsed = false;
    switch (decode_state)
    {
    case WAIT_FOR_HEAD:
    {
        if (c == '$')
        {
            data_pos = 0;
            frame_checksum = 0;
            checksum_data_pos = 0;
            checksum_flag = false;
            internal_checksum = 0;
            memset(checksum_buffer, 0, 2);

            data[data_pos++] = (char)c;
            //printf("%c",data[data_pos-1]);
            decode_state = WAIT_FOR_DATA;
        }
        break;
    }
    case WAIT_FOR_DATA:
    {
        data[data_pos++] = (char)c;
        //printf("%c",data[data_pos-1]);
        if(checksum_flag == true)
        {
            checksum_buffer[checksum_data_pos++] = c;
            if(checksum_data_pos == 2)
            {
                data[data_pos]='\0';
                //printf("%s\n",data);
                checksum_buffer[0] = Ascii2Hex(checksum_buffer[0]);
                checksum_buffer[1] = Ascii2Hex(checksum_buffer[1]);
                frame_checksum = (unsigned char)(checksum_buffer[0] << 4 | checksum_buffer[1]);
                internal_checksum = CalcGPGGAChecksum(data, data_pos-3);
                //printf("check1=%.2X,check2=%.2X\n\n",frame_checksum,internal_checksum);
                new_frame_parsed = true;
                decode_state = WAIT_FOR_HEAD;
                break;
            }
        }

        if (c == '*')
            checksum_flag = true;
        break;
    }
    }

    if (new_frame_parsed)
    {
        if (frame_checksum == internal_checksum)
//        if(true)
        {
            ConstructStatusMessage(&msg->status_msg);
        }
    }
    return new_frame_parsed;
}

bool ConstructStatusMessage(GnssStatusMessage *msg)
{
    if (msg == NULL)
        return false;

    // $GPGGA, $GPRMC, $GTIMU, $GPFPD
    if (data[3] == 'G' && data[4] == 'G' && data[5] == 'A')
    {
        msg->msg_type = GnssGPGGAMsg;
        strcpy(msg->gnss_status_msg, data);
    }
    else if (data[3] == 'R' && data[4] == 'M' && data[5] == 'C')
    {
        msg->msg_type = GnssGPRMCMsg;
        strcpy(msg->gnss_status_msg, data);
    }
    else if (data[3] == 'F' && data[4] == 'P' && data[5] == 'D')
    {
        msg->msg_type = GnssGPFPDMsg;
        strcpy(msg->gnss_status_msg, data);
    }
    else if (data[3] == 'I' && data[4] == 'M' && data[5] == 'U')
    {
        msg->msg_type = GnssGTIMUMsg;
        strcpy(msg->gnss_status_msg, data);
    }

    return true;
}
