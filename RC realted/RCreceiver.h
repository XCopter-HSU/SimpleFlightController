/*
 * HSUM protocol documentation
 *
 * Currently Graupner HoTT serial port settings:
 *  115200bps serial stream, 8 bits, no parity, 1 stop bit
 *  size of each frame: 11..37 bytes
 *  data resolution: 14 bit
 *  frame period: 11ms or 22ms
 *
 * Currently known SUMD/SUMH frame structure:
 *  Section          Byte_Number        Byte_Name      Byte_Value Remark
 *  Header           0                  Vendor_ID      0xA8       Graupner
 *  Header			 1					Status                    0x01       valid and live SUMD data frame
 *                                                     0x81       valid SUMD data frame with
 *                                                                transmitter in fail safe condition
 *                                                     others     invalid frame
 *  Header           2                  N_Channels     0x02..0x20 number of transmitted channels
 *  Data             n*2+1              Channel n MSB  0x00..0xff High Byte of channel n data
 *  Data             n*2+2              Channel n LSB  0x00..0xff Low Byte of channel n data
 *  SUMD_CRC         (N_Channels+1)*2+1 CRC High Byte  0x00..0xff High Byte of 16 Bit CRC
 *  SUMD_CRC         (N_Channels+1)*2+2 CRC Low Byte   0x00..0xff Low Byte of 16 Bit CRC
 *
 * Channel Data Interpretation
 *  Stick Positon    Channel Data Remark
 *  ext. low (-150%) 0x1c20       900�s
 *  low (-100%)      0x2260       1100�s
 *  neutral (0%)     0x2ee0       1500�s
 *  high (100%)      0x3b60       1900�s
 *  ext. high(150%)  0x41a0       2100�s

 * Channel Mapping (not sure)
 *  1 Pitch
 *  2 Aileron
 *  3 Elevator
 *  4 Yaw
 *  5 Aux/Gyro on MX-12
 *  6 ESC
 *  7 Aux/Gyr
 */

#ifndef RCRECEIVER_H_
#define RCRECEIVER_H_

#include <stdint.h>

extern uint16_t rcValue[8]; //extern value will be updated by updateChannelsRC

#define RC_CHANS 8
#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAXCHAN 8
#define SUMD_BUFFSIZE SUMD_MAXCHAN*2 + 5 // 6 channels + 5 -> 17 bytes for 6 channels
#define SUMD_CRC_LENGTH 2



/**
 * Function to initiate UART core
 */
void initRCreceiver();

/**
 * Function to update new rcCommands
 * should be called in every flightcontrol task iterration
 *
 * will update global pointer with 16bit rc channel values
 */
int updateChannelsRC();

//struct SUMD_Frame {
//	uint8_t vendorID;
//	uint8_t status;
//	uint8_t numChannels;
//	//channel_data[0] is MSB of channel 0, channel_data[1] is LSB of channel 0
//	uint16_t channel_data[RC_CHANS];
//	uint8_t crcHighByte;
//	uint8_t crcLowByte;
//}sumdData;

#endif /* RCRECEIVER_H_ */