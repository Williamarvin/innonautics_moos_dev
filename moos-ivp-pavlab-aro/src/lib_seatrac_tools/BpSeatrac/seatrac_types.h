/*===============================================================================
SEATRAC SDK (SOFTWARE DEVELOPMENT KIT)
================================================================================
LICENSE & COPYRIGHT:
The 'SeaTrac SDK' is free software released under the terms of the 'MIT License':

Copyright (c) 2014-2017 Blueprint Design Engineering Ltd (trading as Blueprint Subsea)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

For further details on the license, copyright and software usage, please see
the accompanying ‘readme.txt’ file in the software distribution.
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_TYPES_H
#define SEATRAC_TYPES_H

//==============================================================================
// Import Libraries
//==============================================================================
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//#pragma pack(push, 1)

//==============================================================================
//Determine the type of compiler
//==============================================================================
#if !defined(COMPILER_GCC) && (defined(__GNUC__) || defined(__GNUG__))
    #define COMPILER_GCC
#elif !defined(COMPILER_MSVC) && defined(_MSC_VER)
    #define COMPILER_MSVC
#endif

//==============================================================================
// Fundamental Constants
//==============================================================================
#ifndef FALSE
    #define  FALSE  false                   /* Boolean value FALSE. FALSE is defined always as a zero value. */
#endif

#ifndef TRUE
    #define  TRUE   true                    /* Boolean value TRUE. TRUE is defined always as a non zero value. */
#endif

#ifndef null
    #define  null   0
#endif

#ifndef NULL
    #define  NULL   null
#endif

#ifndef NULLPTR
    #define NULLPTR	((void*)NULL)
#endif

//==============================================================================
// Fundamental Types
//==============================================================================
#ifndef TYPES_H
    //If fundamental types havent been defined, then define them

    typedef signed char			int8;
    typedef signed short		int16;
    typedef signed int			int32;
    typedef signed long long	int64;
    typedef float				single;
    typedef unsigned char		uint8;
    typedef unsigned short		uint16;
    typedef unsigned int		uint32;
    typedef unsigned long long	uint64;

    //Pointer Types
    typedef void*				pointer;
    typedef bool*				pbool;
    typedef char*				pchar;
    typedef double*				pdouble;
    typedef float*				pfloat;
    typedef int8*				pint8;
    typedef int16*				pint16;
    typedef int32*				pint32;
    typedef int64*				pint64;
    typedef float*				psingle;
    typedef uint8*				puint8;
    typedef uint16*				puint16;
    typedef uint32*				puint32;
    typedef uint64*				puint64;
#endif

//==============================================================================
// Helper and Bit Manipulation Macros
//==============================================================================
#ifndef MACROS_H
    //If Macro's haven't been defined, then define them

    #define TOBOOL(x)                       ((x) != FALSE)          /*!< Macro that removes compiler warning C4800 "forcing value to bool 'true' or 'false' (performance warning)" */

    #define BIT(b)                          (1u << ((uint8)b))

    #define SET_BITS(r, bmsk)               (r |= (bmsk))
    #define CLR_BITS(r, bmsk)               (r &= ~(bmsk))

    #define SET_BIT(r, b)                   SET_BITS(r, BIT(b))
    #define CLR_BIT(r, b)                   CLR_BITS(r, BIT(b))

    #define IS_BITS_MATCH(r, bmsk, value)   ((r & (bmsk)) == value)
    #define IS_BITS_SET(r, bmsk)            IS_BITS_MATCH(r, bmsk, bmsk)
    #define IS_BITS_CLR(r, bmsk)            IS_BITS_MATCH(r, bmsk, 0)

    #define IS_BIT_SET(r, b)                TOBOOL(r & BIT(b))
    #define IS_BIT_CLR(r, b)                TOBOOL(!IS_BIT_SET(r,b))
#endif

//==============================================================================
// Constants
//==============================================================================
#define ST_DEFS_VERSION                     1

//------------------------------------------------------------------------------
//Define part numbers of Seatrac Beacon hardware
#define ST_HARDWARE_PARTNUMBER_X010         977
#define ST_HARDWARE_PARTNUMBER_X110         843
#define ST_HARDWARE_PARTNUMBER_X150         795

//Define the minimum supported firmware that this library is compatible with
//(Beacons with main application firmware less than this should be upgraded
//to the latest firmware release)
#define ST_FIRMWARE_MAIN_VERSION_MAJOR      2
#define ST_FIRMWARE_MAIN_VERSION_MINOR      4

//------------------------------------------------------------------------------
/*! The number of bytes (not hex char pairs) that can be received in a message payload */
#define ST_MSG_LEN                          2048

/*! Define the character used to start messages sent from the command processor */
#define ST_ENCODE_HEADER_CHAR               '#'

/*! Define the character used to identify the start of messages to the command processor */
#define ST_DECODE_HEADER_CHAR               '$'

//------------------------------------------------------------------------------
/*!< Define the number of supported beacon ID codes */
#define ST_BEACONS                          16

//------------------------------------------------------------------------------
/*!< Define the maximum number of bytes a SeaTrac acoustic message payload can hold */
#define ST_ACOMSG_PAYLOAD_LEN               31

//------------------------------------------------------------------------------
//Define parameters for data types related to the USBL engine
#define ST_USBL_CHANNELS                    4
#define ST_USBL_BASELINES                   6
#define ST_USBL_XCOR_DATA_MAX               256

//------------------------------------------------------------------------------
//Bit definitions for the flags field in a TSeatracAcoFix.Flags field.
#define ST_ACOFIX_FLAGS_RANGE_VALID_BIT     0
#define ST_ACOFIX_FLAGS_USBL_VALID_BIT      1
#define ST_ACOFIX_FLAGS_POS_VALID_BIT       2
#define ST_ACOFIX_FLAGS_POS_ENHANCED_BIT    3
#define ST_ACOFIX_FLAGS_POS_FLT_ERROR_BIT   4

//------------------------------------------------------------------------------
//Bit definitions for the Status Output flags field in a TSeatracStatus.Flags field.
#define ST_STATUS_FLAGS_ENVIRONMENT_BIT     0
#define ST_STATUS_FLAGS_ATTITUDE_BIT        1
#define ST_STATUS_FLAGS_MAG_CAL_BIT         2
#define ST_STATUS_FLAGS_ACC_CAL_BIT         3
#define ST_STATUS_FLAGS_AHRS_RAW_BIT        4
#define ST_STATUS_FLAGS_AHRS_COMP_BIT       5

//------------------------------------------------------------------------------
//Bit definitions for the Environment flags field in a TSeatracSettings.EnvFlags field.
#define ST_ENV_FLAGS_AUTO_VOS_BIT           0
#define ST_ENV_FLAGS_AUTO_PRESSURE_BIT      1

//------------------------------------------------------------------------------
//Bit definitions for the Ahrs flags field in a TSeatracSettings.AhrsFlags field.
#define ST_AHRS_FLAGS_AUTO_CALC_MAG_BIT     0

//------------------------------------------------------------------------------
//Bit definitions for the Xcvr flags field in a TSeatracSettings.XcvrFlags field.
#define ST_XCVR_FLAGS_USBL_USE_AHRS_BIT     0
#define ST_XCVR_FLAGS_POSFLT_ENABLE_BIT     1
#define ST_XCVR_FLAGS_TX_MSGCTRL0_BIT		3
#define ST_XCVR_FLAGS_TX_MSGCTRL1_BIT		4
#define ST_XCVR_FLAGS_USBL_MSGS_BIT         5
#define ST_XCVR_FLAGS_FIX_MSGS_BIT          6
#define ST_XCVR_FLAGS_DIAG_MSGS_BIT         7

#define ST_XCVR_FLAGS_TX_MSGCTRL_SHIFT		ST_XCVR_FLAGS_TX_MSGCTRL0_BIT
#define ST_XCVR_FLAGS_TX_MSGCTRL0_MASK		BIT(ST_XCVR_FLAGS_TX_MSGCTRL0_BIT)
#define ST_XCVR_FLAGS_TX_MSGCTRL1_MASK		BIT(ST_XCVR_FLAGS_TX_MSGCTRL1_BIT)
#define ST_XCVR_FLAGS_TX_MSGCTRL_MASK		(ST_XCVR_FLAGS_TX_MSGCTRL1_MASK | ST_XCVR_FLAGS_TX_MSGCTRL0_MASK)

//------------------------------------------------------------------------------
//Define the number of bytes in a NAV data payload
#define ST_NAV_PAYLOAD_LEN                  (ST_ACOMSG_PAYLOAD_LEN - 1)

//Bit definitions for Nav Query Flags.
#define ST_NAVQUERY_FLAGS_DEPTH_BIT         0
#define ST_NAVQUERY_FLAGS_SUPPLY_BIT        1
#define ST_NAVQUERY_FLAGS_TEMP_BIT          2
#define ST_NAVQUERY_FLAGS_ATTITUDE_BIT      3
#define ST_NAVQUERY_FLAGS_DATA_BIT          7

//Define bit masks for the above bit constants
#define ST_NAVQUERY_FLAGS_DEPTH_MASK        BIT(ST_NAVQUERY_FLAGS_DEPTH_BIT)
#define ST_NAVQUERY_FLAGS_SUPPLY_MASK       BIT(ST_NAVQUERY_FLAGS_SUPPLY_BIT)
#define ST_NAVQUERY_FLAGS_TEMP_MASK         BIT(ST_NAVQUERY_FLAGS_TEMP_BIT)
#define ST_NAVQUERY_FLAGS_ATTITUDE_MASK     BIT(ST_NAVQUERY_FLAGS_ATTITUDE_BIT)
#define ST_NAVQUERY_FLAGS_DATA_MASK         BIT(ST_NAVQUERY_FLAGS_DATA_BIT)

//------------------------------------------------------------------------------
//Bit Flags for enabling features of the DiagLog event
#define SEATRAC_DIAGLOG_ENCODE_BIT      	0
#define SEATRAC_DIAGLOG_DECODE_BIT     		1
#define SEATRAC_DIAGLOG_EXECUTE_BIT     	2
#define SEATRAC_DIAGLOG_EXECUTED_BIT    	3
#define SEATRAC_DIAGLOG_SYS_BIT         	4
#define SEATRAC_DIAGLOG_STATUS_BIT      	5
#define SEATRAC_DIAGLOG_SETTINGS_BIT    	6
#define SEATRAC_DIAGLOG_CAL_BIT             7
#define SEATRAC_DIAGLOG_XCVR_BIT        	8
#define SEATRAC_DIAGLOG_PING_BIT        	9
#define SEATRAC_DIAGLOG_ECHO_BIT        	10
#define SEATRAC_DIAGLOG_DAT_BIT         	11
#define SEATRAC_DIAGLOG_NAV_BIT         	12
#define SEATRAC_DIAGLOG_CFG_BIT         	14

//Define bit masks for the above bit constants
#define SEATRAC_DIAGLOG_ENCODE_MASK     	BIT(SEATRAC_DIAGLOG_ENCODE_BIT)
#define SEATRAC_DIAGLOG_DECODE_MASK     	BIT(SEATRAC_DIAGLOG_DECODE_BIT)
#define SEATRAC_DIAGLOG_EXECUTE_MASK    	BIT(SEATRAC_DIAGLOG_EXECUTE_BIT)
#define SEATRAC_DIAGLOG_EXECUTED_MASK   	BIT(SEATRAC_DIAGLOG_EXECUTED_BIT)
#define SEATRAC_DIAGLOG_SYS_MASK        	BIT(SEATRAC_DIAGLOG_SYS_BIT)
#define SEATRAC_DIAGLOG_STATUS_MASK     	BIT(SEATRAC_DIAGLOG_STATUS_BIT)
#define SEATRAC_DIAGLOG_SETTINGS_MASK    	BIT(SEATRAC_DIAGLOG_SETTINGS_BIT)
#define SEATRAC_DIAGLOG_CAL_MASK            BIT(SEATRAC_DIAGLOG_CAL_BIT)
#define SEATRAC_DIAGLOG_XCVR_MASK       	BIT(SEATRAC_DIAGLOG_XCVR_BIT)
#define SEATRAC_DIAGLOG_PING_MASK       	BIT(SEATRAC_DIAGLOG_PING_BIT)
#define SEATRAC_DIAGLOG_ECHO_MASK       	BIT(SEATRAC_DIAGLOG_ECHO_BIT)
#define SEATRAC_DIAGLOG_DAT_MASK        	BIT(SEATRAC_DIAGLOG_DAT_BIT)
#define SEATRAC_DIAGLOG_NAV_MASK        	BIT(SEATRAC_DIAGLOG_NAV_BIT)
#define SEATRAC_DIAGLOG_CFG_MASK        	BIT(SEATRAC_DIAGLOG_CFG_BIT)

#define SEATRAC_DIAGLOG_ALL_MASK        	0xFFFFFFFF
#define SEATRAC_DIAGLOG_PROTOCOLS_MASK      (SEATRAC_DIAGLOG_PING_MASK | SEATRAC_DIAGLOG_ECHO_MASK | SEATRAC_DIAGLOG_DAT_MASK | SEATRAC_DIAGLOG_NAV_MASK | SEATRAC_DIAGLOG_CFG_MASK)
#define SEATRAC_DIAGLOG_STANDARD_MASK       (SEATRAC_DIAGLOG_SYS_MASK | SEATRAC_DIAGLOG_SETTINGS_MASK | SEATRAC_DIAGLOG_XCVR_MASK | SEATRAC_DIAGLOG_PROTOCOLS_MASK)

//==============================================================================
// Enumerations (for C and C++11 compatability)
//==============================================================================
#ifdef __cplusplus
    //! Beacon Identifier enumeration
    enum ESeatracBeaconId : uint8 {
        ST_BEACON_ALL = 0,                  				/*!< Used as a broadcast-to-all address for message types that support that function */
        ST_BEACON_1 = 1,
        ST_BEACON_2 = 2,
        ST_BEACON_3 = 3,
        ST_BEACON_4 = 4,
        ST_BEACON_5 = 5,
        ST_BEACON_6 = 6,
        ST_BEACON_7 = 7,
        ST_BEACON_8 = 8,
        ST_BEACON_9 = 9,
        ST_BEACON_10 = 10,
        ST_BEACON_11 = 11,
        ST_BEACON_12 = 12,
        ST_BEACON_13 = 13,
        ST_BEACON_14 = 14,
        ST_BEACON_15 = 15,
        ST_BEACON_UNKNOWN = 0xFF,
    };
#else
    //! Beacon Identifier
    typedef uint8 ESeatracBeaconId;

    #define ST_BEACON_ALL					0      			/*!< Used as a broadcast-to-all address for message types that support that function */
    #define ST_BEACON_1						1
    #define ST_BEACON_2						2
    #define ST_BEACON_3						3
    #define ST_BEACON_4						4
    #define ST_BEACON_5						5
    #define ST_BEACON_6						6
    #define ST_BEACON_7						7
    #define ST_BEACON_8						8
    #define ST_BEACON_9						9
    #define ST_BEACON_10					10
    #define ST_BEACON_11					11
    #define ST_BEACON_12					12
    #define ST_BEACON_13					13
    #define ST_BEACON_14					14
    #define ST_BEACON_15					15
    #define ST_BEACON_UNKNOWN				0xFF
#endif

//------------------------------------------------------------------------------
#ifdef __cplusplus
    enum ESeatracAcoMsgType : uint8 {
        ST_AMSG_OWAY = 0x0,                 				/*!< Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. No USBL information is sent. */
        ST_AMSG_OWAYU = 0x1,                				/*!< Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. Additionally, the message is sent with USBL acoustic information allowing an incoming bearing to be determined by USBL receivers, although range cannot be provided. */
        ST_AMSG_REQ = 0x2,                  				/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. No USBL information is requested. */
        ST_AMSG_RESP = 0x3,                 				/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). No USBL information is returned. */
        ST_AMSG_REQU = 0x4,                 				/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with USBL acoustic information allowing a position fix to be computed by USBL receivers through the range and incoming signal angle. */
        ST_AMSG_RESPU = 0x5,                				/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with USBL acoustic information allowing the position of the sender to be determined through the range and incoming signal angle. */
        ST_AMSG_REQX = 0x6,                	 				/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with extended Depth and USBL acoustic information allowing a more accurate position fix to be computed by USBL receivers through the range, remote depth and incoming signal angle. */
        ST_AMSG_RESPX = 0x7,                				/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with extended depth and USBL acoustic information allowing a more accurate position of the sender to be determined through the range, remote depth and incoming signal angle. */
        ST_AMSG_UNKNOWN = 0xFF,             				/*!< This value is NEVER used to specify a message type when sending Acoustic Messages. However, on occasions certain structures need to specify â€œNo Message Typeâ€ (for example see ACOFIX_T), and this value is used as an OUTPUT ONLY to indicate this. */
    };
#else
    typedef uint8 ESeatracAcoMsgType;

    #define ST_AMSG_OWAY					0x0     		/*!< Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. No USBL information is sent. */
    #define ST_AMSG_OWAYU					0x1        		/*!< Indicates an acoustic message is sent One-Way, and does not require a response. One-Way messages may also be broadcast to all beacons if required. Additionally, the message is sent with USBL acoustic information allowing an incoming bearing to be determined by USBL receivers, although range cannot be provided. */
    #define ST_AMSG_REQ						0x2        		/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. No USBL information is requested. */
    #define ST_AMSG_RESP					0x3        		/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). No USBL information is returned. */
    #define ST_AMSG_REQU					0x4        		/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with USBL acoustic information allowing a position fix to be computed by USBL receivers through the range and incoming signal angle. */
    #define ST_AMSG_RESPU					0x5         	/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with USBL acoustic information allowing the position of the sender to be determined through the range and incoming signal angle. */
    #define ST_AMSG_REQX					0x6         	/*!< Indicates an acoustic message is sent as a Request type. This requires the receiver to generate and return a Response (MSG_RESP) message. Additionally, the Response message should be returned with extended Depth and USBL acoustic information allowing a more accurate position fix to be computed by USBL receivers through the range, remote depth and incoming signal angle. */
    #define ST_AMSG_RESPX					0x7         	/*!< Indicates an acoustic message is sent as a Response to a previous Request message (MSG_REQ). Additionally, the message is sent with extended depth and USBL acoustic information allowing a more accurate position of the sender to be determined through the range, remote depth and incoming signal angle. */
    #define ST_AMSG_UNKNOWN					0xFF        	/*!< This value is NEVER used to specify a message type when sending Acoustic Messages. However, on occasions certain structures need to specify “No Message Type” (for example see ACOFIX_T), and this value is used as an OUTPUT ONLY to indicate this. */
#endif

//------------------------------------------------------------------------------
#ifdef __cplusplus
    enum ESeatracAcoPayloadType : uint8 {
        ST_APLOAD_PING = 0x0,                   			/*!< Specified an acoustic message payload should be interpreted by the PING protocol handler. PING messages provide the simplest (and quickest) method of validating the presence of a beacon, and determining its position. */
        ST_APLOAD_ECHO = 0x1,                   			/*!< Specified an acoustic message payload should be interpreted by the ECHO protocol handler. ECHO messages allow the function and reliability of a beacon to be tested, by requesting the payload contents of the message be returned back to the sender. */
        ST_APLOAD_NAV = 0x2,                    			/*!< Specified an acoustic message payload should be interpreted by the NAV (Navigation) protocol handler. NAV messages allow tracking and navigation systems to be built that use enhanced positioning and allow remote parameters of beacons (such as heading, attitude, water temperature etc) to be queried. */
        ST_APLOAD_DAT = 0x3,                    			/*!< Specified an acoustic message payload should be interpreted by the DAT (Datagram) protocol handler. DAT messages for the simplest method of data exchange between beacons, and provide a method of acknowledging data reception. */
        ST_APLOAD_DEX = 0x4,                  			  	/*!< Specified an acoustic message payload should be interpreted by the DEX (Data Exchange) protocol handler. DEX messages implement a protocol that allows robust bi-directional socket based data exchange with timeouts, acknowledgments and retry schemes. */
        ST_APLOAD_UNKNOWN = 0xFF,             			  	/*!< This value is only used by the SeaTrac interface to specify a payload type when an error occurs */
    };
#else
    typedef uint8 ESeatracAcoPayloadType;

    #define ST_APLOAD_PING    				0x0,          	/*!< Specified an acoustic message payload should be interpreted by the PING protocol handler. PING messages provide the simplest (and quickest) method of validating the presence of a beacon, and determining its position. */
    #define ST_APLOAD_ECHO      			0x1,            /*!< Specified an acoustic message payload should be interpreted by the ECHO protocol handler. ECHO messages allow the function and reliability of a beacon to be tested, by requesting the payload contents of the message be returned back to the sender. */
    #define ST_APLOAD_NAV       			0x2,            /*!< Specified an acoustic message payload should be interpreted by the NAV (Navigation) protocol handler. NAV messages allow tracking and navigation systems to be built that use enhanced positioning and allow remote parameters of beacons (such as heading, attitude, water temperature etc) to be queried. */
    #define ST_APLOAD_DAT       			0x3,            /*!< Specified an acoustic message payload should be interpreted by the DAT (Datagram) protocol handler. DAT messages for the simplest method of data exchange between beacons, and provide a method of acknowledging data reception. */
    #define ST_APLOAD_DEX       			0x4,            /*!< Specified an acoustic message payload should be interpreted by the DEX (Data Exchange) protocol handler. DEX messages implement a protocol that allows robust bi-directional socket based data exchange with timeouts, acknowledgments and retry schemes. */
    #define ST_APLOAD_UNKNOWN   			0xFF,           /*!< This value is only used by the SeaTrac interface to specify a payload type when an error occurs */
#endif

//------------------------------------------------------------------------------
#ifdef __cplusplus
    enum ESeatracCmdId : uint8 {
        ST_CID_INVALID = 0x00,
        // System Messages
        ST_CID_SYS_ALIVE = 0x01,                			/*!< Command sent to receive a simple alive message from the beacon. */
        ST_CID_SYS_INFO = 0x02,                 			/*!< Command sent to receive hardware & firmware identification information. */
        ST_CID_SYS_REBOOT = 0x03,               			/*!< Command sent to soft reboot the beacon. */
        ST_CID_SYS_ENGINEERING = 0x04,          			/*!< Command sent to perform engineering actions. */
        // Firmware Programming Messages
        ST_CID_PROG_INIT = 0x0D,                			/*!< Command sent to initialise a firmware programming sequence. */
        ST_CID_PROG_BLOCK = 0x0E,               			/*!< Command sent to transfer a firmware programming block. */
        ST_CID_PROG_UPDATE = 0x0F,              			/*!< Command sent to update the firmware once program transfer has completed. */
        // Status Messages
        ST_CID_STATUS = 0x10,                   			/*!< Command sent to request the current system status (AHRS, Depth, Temp, etc). */
        ST_CID_STATUS_CFG_GET = 0x11,           			/*!< Command sent to retrieve the configuration of the status system (message content and auto-output interval). */
        ST_CID_STATUS_CFG_SET = 0x12,           			/*!< Command sent to set the configuration of the status system (message content and auto-output interval). */
        // Settings Messages
        ST_CID_SETTINGS_GET = 0x15,             			/*!< Command sent to retrieve the working settings in use on the beacon. */
        ST_CID_SETTINGS_SET = 0x16,             			/*!< Command sent to set the working settings and apply them. They are NOT saved to permanent memory until ST_CID_ SETTINGS_SAVE is issued. The device will need to be rebooted after this to apply some of the changes. */
        ST_CID_SETTINGS_LOAD = 0x17,            			/*!< Command sent to load the working settings from permanent storage and apply them. Not all settings can be loaded and applied as they only affect the device on start-up. */
        ST_CID_SETTINGS_SAVE = 0x18,            			/*!< Command sent to save the working settings into permanent storage. */
        ST_CID_SETTINGS_RESET = 0x19,           			/*!< Command sent to restore the working settings to defaults, store them into permanent memory and apply them. */
		ST_CID_CONFIG_GET = 0x1A,							/*!< Command sent to receive a configuration message, containing Pressure Info and USBL calibration */
        // Calibration Messages
        ST_CID_CAL_ACTION = 0x20,               			/*!< Command sent to perform specific calibration actions. */
        ST_CID_CAL_AHRS_GET = 0x21,             			/*!< Command sent to retrieve the current AHRS calibration. */
        ST_CID_CAL_AHRS_SET = 0x22,             			/*!< Command sent to set the contents of the current AHRS calibration (and store to memory) */
        // Acoustic Transceiver Messages
        ST_CID_XCVR_ANALYSE = 0x30,             			/*!< Command sent to instruct the receiver to perform a noise analysis and report the results. */
        ST_CID_XCVR_TX_MSG = 0x31,              			/*!< Message sent when the transceiver transmits a message. */
        ST_CID_XCVR_RX_ERR = 0x32,              			/*!< Message sent when the transceiver receiver encounters an error. */
        ST_CID_XCVR_RX_MSG = 0x33,              			/*!< Message sent when the transceiver receives a message (not requiring a response). */
        ST_CID_XCVR_RX_REQ = 0x34,              			/*!< Message sent when the transceiver receives a request (requiring a response). */
        ST_CID_XCVR_RX_RESP = 0x35,             			/*!< Message sent when the transceiver receives a response (to a transmitted request). */
        ST_CID_XCVR_RX_UNHANDLED =	0x37,       			/*!< Message sent when a message has been received but not handled by the protocol stack. */
        ST_CID_XCVR_USBL =	0x38,               			/*!< Message sent when a USBL signal is decoded into an angular bearing. */
        ST_CID_XCVR_FIX =	0x39,               			/*!< Message sent when the transceiver gets a position/range fix on a beacon from a request/response. */
        ST_CID_XCVR_STATUS =	0x3A,           			/*!< Message sent to query the current transceiver state. */
		ST_CID_XCVR_TX_MSGCTRL_SET = 0x3B,					/*!< Message sent to set or query the Transmit Mode (BlockAll, BlockResponse, AllowAll) */
        // PING Protocol Messages
        ST_CID_PING_SEND = 0x40,                			/*!< Command sent to transmit a PING message. */
        ST_CID_PING_REQ = 0x41,                 			/*!< Message sent when a PING request is received. */
        ST_CID_PING_RESP = 0x42,                			/*!< Message sent when a PING response is received, or timeout occurs, with the echo response data. */
        ST_CID_PING_ERROR = 0x43,               			/*!< Message sent when a PING response error/timeout occurs. */
        // ECHO Protocol Messages
        ST_CID_ECHO_SEND = 0x48,                			/*!< Command sent to transmit an ECHO message. */
        ST_CID_ECHO_REQ = 0x49,                 			/*!< Message sent when an ECHO request is received. */
        ST_CID_ECHO_RESP = 0x4A,                			/*!< Message sent when an ECHO response is received, or timeout occurs, with the echo response data. */
        ST_CID_ECHO_ERROR = 0x4B,               			/*!< Message sent when an ECHO response error/timeout occurs. */
        // NAV Protocol Messages
        ST_CID_NAV_QUERY_SEND = 0x50,           			/*!< Message sent to query navigation information from a remote beacon. */
        ST_CID_NAV_QUERY_REQ = 0x51,            			/*!< Message sent from a beacon that receives a NAV_QUERY. */
        ST_CID_NAV_QUERY_RESP = 0x52,           			/*!< Message generated when the beacon received a response to a NAV_QUERY. */
        ST_CID_NAV_ERROR = 0x53,                			/*!< Message generated if there is a problem with a NAV_QUERY - i.e. timeout etc. */
        ST_CID_NAV_QUEUE_SET = 0x58,                        /*!< Message sent to queue a packet of data ready for remote interrogation */
        ST_CID_NAV_QUEUE_CLR = 0x59,                        /*!< Message sent to query the packet queue */
        ST_CID_NAV_QUEUE_STATUS = 0x5A,                     /*!< Message sent to query the status of queued packets for each beacon */
        ST_CID_NAV_STATUS_SEND = 0x5B,                      /*!< Message issued to broadcast status information to all other beacons */
        ST_CID_NAV_STATUS_RECEIVE = 0x5C,                   /*!< Message generated when a beacon receives a NAV_STATUS message */
        // DAT Protocol Messages
        ST_CID_DAT_SEND = 0x60,                 			/*!< Message sent to transmit a datagram to another beacon */
        ST_CID_DAT_RECEIVE = 0x61,              			/*!< Message generated when a beacon receives a datagram. */
        ST_CID_DAT_ERROR = 0x63,                			/*!< Message generated when a beacon response error/timeout occurs for ACKs. */
        ST_CID_DAT_QUEUE_SET = 0x64,            			/*!< Message sent to set the contents of the packet data queue. */
        ST_CID_DAT_QUEUE_CLR = 0x65,            			/*!< Message sent to clear the contents of the packet data queue. */
        ST_CID_DAT_QUEUE_STATUS = 0x66,         			/*!< Message sent to obtain the current status of the packet data queue. */
		//CFG Protocol Messages
		ST_CID_CFG_BEACON_GET = 0x80,						/*!< Message sent to read the current remote beacon transceiver setup*/
		ST_CID_CFG_BEACON_SET = 0x81,						/*!< Message sent to set new values for the remote beacom transceiver */
		ST_CID_CFG_BEACON_RESP = 0x82,						/*!< Message sent when a response of the configuration from the remote beacon is received back (as a MSG_OWAY type) */
    };
#else
    typedef uint8 ESeatracCmdId;

    #define ST_CID_INVALID       			0x00
    // System Messages
    #define ST_CID_SYS_ALIVE            	0x01       		/*!< Command sent to receive a simple alive message from the beacon. */
    #define ST_CID_SYS_INFO            		0x02        	/*!< Command sent to receive hardware & firmware identification information. */
    #define ST_CID_SYS_REBOOT           	0x03        	/*!< Command sent to soft reboot the beacon. */
    #define ST_CID_SYS_ENGINEERING      	0x04        	/*!< Command sent to perform engineering actions. */
    // Firmware Programming Messages
    #define ST_CID_PROG_INIT            	0x0D        	/*!< Command sent to initialise a firmware programming sequence. */
    #define ST_CID_PROG_BLOCK           	0x0E        	/*!< Command sent to transfer a firmware programming block. */
    #define ST_CID_PROG_UPDATE          	0x0F        	/*!< Command sent to update the firmware once program transfer has completed. */
    // Status Messages
    #define ST_CID_STATUS            		0x10        	/*!< Command sent to request the current system status (AHRS, Depth, Temp, etc). */
    #define ST_CID_STATUS_CFG_GET       	0x11            /*!< Command sent to retrieve the configuration of the status system (message content and auto-output interval). */
    #define ST_CID_STATUS_CFG_SET       	0x12            /*!< Command sent to set the configuration of the status system (message content and auto-output interval). */
    // Settings Messages
    #define ST_CID_SETTINGS_GET         	0x15        	/*!< Command sent to retrieve the working settings in use on the beacon. */
    #define ST_CID_SETTINGS_SET         	0x16        	/*!< Command sent to set the working settings and apply them. They are NOT saved to permanent memory until ST_CID_ SETTINGS_SAVE is issued. The device will need to be rebooted after this to apply some of the changes. */
    #define ST_CID_SETTINGS_LOAD        	0x17        	/*!< Command sent to load the working settings from permanent storage and apply them. Not all settings can be loaded and applied as they only affect the device on start-up. */
    #define ST_CID_SETTINGS_SAVE        	0x18        	/*!< Command sent to save the working settings into permanent storage. */
    #define ST_CID_SETTINGS_RESET       	0x19        	/*!< Command sent to restore the working settings to defaults, store them into permanent memory and apply them. */
	#define ST_CID_CONFIG_GET				0x1A			/*!< Command sent to receive a configuration message, containing Pressure Info and USBL calibration */
    // Calibration Messages
    #define ST_CID_CAL_ACTION           	0x20        	/*!< Command sent to perform specific calibration actions. */
    #define ST_CID_AHRS_CAL_GET         	0x21        	/*!< Command sent to retrieve the current AHRS calibration. */
    #define ST_CID_AHRS_CAL_SET         	0x22        	/*!< Command sent to set the contents of the current AHRS calibration (and store to memory) */
    // Acoustic Transceiver Messages
    #define ST_CID_XCVR_ANALYSE         	0x30        	/*!< Command sent to instruct the receiver to perform a noise analysis and report the results. */
    #define ST_CID_XCVR_TX_MSG          	0x31        	/*!< Message sent when the transceiver transmits a message. */
    #define ST_CID_XCVR_RX_ERR          	0x32        	/*!< Message sent when the transceiver receiver encounters an error. */
    #define ST_CID_XCVR_RX_MSG          	0x33        	/*!< Message sent when the transceiver receives a message (not requiring a response). */
    #define ST_CID_XCVR_RX_REQ          	0x34        	/*!< Message sent when the transceiver receives a request (requiring a response). */
    #define ST_CID_XCVR_RX_RESP         	0x35        	/*!< Message sent when the transceiver receives a response (to a transmitted request). */
    #define ST_CID_XCVR_RX_UNHANDLED		0x37        	/*!< Message sent when a message has been received but not handled by the protocol stack. */
    #define ST_CID_XCVR_USBL				0x38        	/*!< Message sent when a USBL signal is decoded into an angular bearing. */
    #define ST_CID_XCVR_FIX					0x39        	/*!< Message sent when the transceiver gets a position/range fix on a beacon from a request/response. */
    #define ST_CID_XCVR_STATUS				0x3A        	/*!< Message sent to query the current transceiver state. */
	#define ST_CID_XCVR_TX_MSGCTRL_SET		0x3B			/*!< Message sent to set or query the Transmit Mode (BlockAll, BlockResponse, AllowAll) */
    // PING Protocol Messages
    #define ST_CID_PING_SEND            	0x40        	/*!< Command sent to transmit a PING message. */
    #define ST_CID_PING_REQ            		0x41        	/*!< Message sent when a PING request is received. */
    #define ST_CID_PING_RESP            	0x42        	/*!< Message sent when a PING response is received, or timeout occurs, with the echo response data. */
    #define ST_CID_PING_ERROR           	0x43        	/*!< Message sent when a PING response error/timeout occurs. */
    // ECHO Protocol Messages
    #define ST_CID_ECHO_SEND            	0x48        	/*!< Command sent to transmit an ECHO message. */
    #define ST_CID_ECHO_REQ            		0x49        	/*!< Message sent when an ECHO request is received. */
    #define ST_CID_ECHO_RESP            	0x4A        	/*!< Message sent when an ECHO response is received, or timeout occurs, with the echo response data. */
    #define ST_CID_ECHO_ERROR           	0x4B        	/*!< Message sent when an ECHO response error/timeout occurs. */
    // NAV Protocol Messages
    #define ST_CID_NAV_QUERY_SEND       	0x50        	/*!< Message sent to query navigation information from a remote beacon. */
    #define ST_CID_NAV_QUERY_REQ        	0x51        	/*!< Message sent from a beacon that receives a NAV_QUERY. */
    #define ST_CID_NAV_QUERY_RESP       	0x52        	/*!< Message generated when the beacon received a response to a NAV_QUERY. */
    #define ST_CID_NAV_ERROR            	0x53        	/*!< Message generated if there is a problem with a NAV_QUERY - i.e. timeout etc. */
    #define ST_CID_NAV_QUEUE_SET			0x58            /*!< Message sent to queue a packet of data ready for remote interrogation */
    #define ST_CID_NAV_QUEUE_CLR			0x59            /*!< Message sent to query the packet queue */
    #define ST_CID_NAV_QUEUE_STATUS			0x5A            /*!< Message sent to query the status of queued packets for each beacon */
    #define ST_CID_NAV_STATUS_SEND			0x5B            /*!< Message issued to broadcast status information to all other beacons */
    #define ST_CID_NAV_STATUS_RECEIVE		0x5C            /*!< Message generated when a beacon receives a NAV_STATUS message */
    // DAT Protocol Messages
    #define ST_CID_DAT_SEND            		0x60        	/*!< Message sent to transmit a datagram to another beacon */
    #define ST_CID_DAT_RECEIVE          	0x61        	/*!< Message generated when a beacon receives a datagram. */
    #define ST_CID_DAT_ERROR            	0x63        	/*!< Message generated when a beacon response error/timeout occurs for ACKs. */
    #define ST_CID_DAT_QUEUE_SET        	0x64        	/*!< Message sent to set the contents of the packet data queue. */
    #define ST_CID_DAT_QUEUE_CLR        	0x65        	/*!< Message sent to clear the contents of the packet data queue. */
    #define ST_CID_DAT_QUEUE_STATUS     	0x66        	/*!< Message sent to obtain the current status of the packet data queue. */
	// CFG Protocol Messages
	#define CID_CFG_BEACON_GET				0x80			/*!< Message sent to read the current remote beacon transceiver setup*/
	#define CID_CFG_BEACON_SET				0x81			/*!< Message sent to set new values for the remote beacom transceiver */
	#define CID_CFG_BEACON_RESP				0x82			/*!< Message sent when a response of the configuration from the remote beacon is received back (as a MSG_OWAY type) */
#endif

//------------------------------------------------------------------------------
#ifdef __cplusplus
    enum ESeatracCmdStatus : uint8 {
        //General Status Codes
        ST_CST_OK = 0x00,                       			/*!< Returned if a command or operation is completed successful without error. */
        ST_CST_FAIL = 0x01,                     			/*!< Returned if a command or operation cannot be completed. */
        ST_CST_EEPROM_ERROR = 0x03,             			/*!< Returned if an error occurs while reading or writing EEPROM data. */
        //Command Processor Status Codes
        ST_CST_CMD_PARAM_MISSING = 0x04,        			/*!< Returned if a command message is given that does not have enough defined fields for the specified CID code. */
        ST_CST_CMD_PARAM_INVALID = 0x05,        			/*!< Returned if a data field in a message does not contain a valid or expected value. */
        //Firmware Programming Status Codes
        ST_CST_PROG_FLASH_ERROR = 0x0A,         			/*!< Returned if an error occurs while writing data into the processors flash memory. */
        ST_CST_PROG_FIRMWARE_ERROR = 0x0B,      			/*!< Returned if firmware cannot be programmed due to incorrect firmware credentials or signature. */
        ST_CST_PROG_SECTION_ERROR = 0x0C,       			/*!< Returned if the firmware cannot be programmed into the specified memory section. */
        ST_CST_PROG_LENGTH_ERROR = 0x0D,        			/*!< Returned if the firmware length is too large to fit into the specified memory section, or not what the current operation is expecting. */
        ST_CST_PROG_DATA_ERROR = 0x0E,          			/*!< Returned if there is an error decoding data in a firmware block. */
        ST_CST_PROG_CHECKSUM_ERROR = 0x0F,      			/*!< Returned if the specified checksum for the firmware does not match the checksum computed prior to performing the update. */
        //Acoustic Transceiver Status Codes
        ST_CST_XCVR_BUSY = 0x30,                			/*!< Returned if the transceiver cannot perform a requested action as it is currently busy (i.e. transmitting a message). */
        ST_CST_XCVR_ID_REJECTED = 0x31,         			/*!< Returned if the received message did not match the specified transceiver ID (and wasnâ€™t a Sent-To-All), and the message has been rejected. */
        ST_CST_XCVR_CSUM_ERROR = 0x32,          			/*!< Returned if received acoustic messageâ€™s checksum was invalid, and the message has been rejected. */
        ST_CST_XCVR_LENGTH_ERROR = 0x33,        			/*!< Returned if an error occurred with message framing, meaning the end of the message has not been received within the expected time. */
        ST_CST_XCVR_RESP_TIMEOUT = 0x34,        			/*!< Returned if the transceiver has sent a request message to a beacon, but no response has been returned within the allotted waiting period. */
        ST_CST_XCVR_RESP_ERROR = 0x35,          			/*!< Returned if the transceiver has send a request message to a beacon, but an error occurred while receiving the response. */
        ST_CST_XCVR_RESP_WRONG = 0x36,          			/*!< Returned if the transceiver has sent a request message to a beacon, but received an unexpected response from another beacon while waiting. */
        ST_CST_XCVR_PLOAD_ERROR = 0x37,         			/*!< Returned by protocol payload decoders, if the payload canâ€™t be parsed correctly. */
        ST_CST_XCVR_STATE_STOPPED = 0x3A,       			/*!< Indicates the transceiver is in a stopped state. */
        ST_CST_XCVR_STATE_IDLE = 0x3B,          			/*!< Indicates the transceiver is in an idle state waiting for reception or transmission to start. */
        ST_CST_XCVR_STATE_TX = 0x3C,            			/*!< Indicates the transceiver is in a transmitting states. */
        ST_CST_XCVR_STATE_REQ = 0x3D,           			/*!< Indicates the transceiver is in a requesting state, having transmitted a message and is waiting for a response to be received. */
        ST_CST_XCVR_STATE_RX = 0x3E,            			/*!< Indicates the transceiver is in a receiving state. */
        ST_CST_XCVR_STATE_RESP = 0x3F,          			/*!< Indicates the transceiver is in a responding state, where a message is being composed and the â€œresponse timeâ€ period is being observed. */
        //DEX Protocol Status Codes
        ST_CST_DEX_SOCKET_ERROR = 0x70,         			/*!< Returned by the DEX protocol handler if an error occurred trying to open, close or access a specified socket ID. */
        ST_CST_DEX_RX_SYNC = 0x71,              			/*!< Returned by the DEX protocol handler when receiver synchronisation has occurred with the socket master and data transfer is ready to commence. */
        ST_CST_DEX_RX_DATA = 0x72,              			/*!< Returned by the DEX protocol handler when data has been received through a socket. */
        ST_CST_DEX_RX_SEQ_ERROR = 0x73,         			/*!< Returned by the DEX protocol handler when data transfer synchronisation has been lost with the socket master. */
        ST_CST_DEX_RX_MSG_ERROR = 0x74,         			/*!< Returned by the DEX protocol handler to indicate an unexpected acoustic message type with the DEX protocol has been received and cannot be processed. */
        ST_CST_DEX_REQ_ERROR = 0x75,            			/*!< Returned by the DEX protocol handler to indicate a error has occurred while responding to a request (i.e. lack of data). */
        ST_CST_DEX_RESP_TMO_ERROR = 0x76,       			/*!< Returned by the DEX protocol handler to indicate a timeout has occurred while waiting for a response back from a remote beacon with requested data. */
        ST_CST_DEX_RESP_MSG_ERROR = 0x77,       			/*!< Returned by the DEX protocol handler to indicate an error has occurred while receiving response back from a remote beacon. */
        ST_CST_DEX_RESP_REMOTE_ERROR = 0x78,    			/*!< Returned by the DEX protocol handler to indicate the remote beacon has encountered an error and cannot return the requested data or perform the required operation. */
    };
#else
    typedef uint8 ESeatracCmdStatus;

    //General Status Codes
    #define ST_CST_OK            			0x00      		/*!< Returned if a command or operation is completed successful without error. */
    #define ST_CST_FAIL            			0x01        	/*!< Returned if a command or operation cannot be completed. */
    #define ST_CST_EEPROM_ERROR         	0x03        	/*!< Returned if an error occurs while reading or writing EEPROM data. */
    //Command Processor Status Codes
    #define ST_CST_CMD_PARAM_MISSING    	0x04        	/*!< Returned if a command message is given that does not have enough defined fields for the specified CID code. */
    #define ST_CST_CMD_PARAM_INVALID    	0x05        	/*!< Returned if a data field in a message does not contain a valid or expected value. */
    //Firmware Programming Status Codes
    #define ST_CST_PROG_FLASH_ERROR     	0x0A        	/*!< Returned if an error occurs while writing data into the processors flash memory. */
    #define ST_CST_PROG_FIRMWARE_ERROR  	0x0B       		/*!< Returned if firmware cannot be programmed due to incorrect firmware credentials or signature. */
    #define ST_CST_PROG_SECTION_ERROR   	0x0C        	/*!< Returned if the firmware cannot be programmed into the specified memory section. */
    #define ST_CST_PROG_LENGTH_ERROR    	0x0D        	/*!< Returned if the firmware length is too large to fit into the specified memory section, or not what the current operation is expecting. */
    #define ST_CST_PROG_DATA_ERROR      	0x0E        	/*!< Returned if there is an error decoding data in a firmware block. */
    #define ST_CST_PROG_CHECKSUM_ERROR  	0x0F       		/*!< Returned if the specified checksum for the firmware does not match the checksum computed prior to performing the update. */
    //Acoustic Transceiver Status Codes
    #define ST_CST_XCVR_BUSY            	0x30        	/*!< Returned if the transceiver cannot perform a requested action as it is currently busy (i.e. transmitting a message). */
    #define ST_CST_XCVR_ID_REJECTED     	0x31        	/*!< Returned if the received message did not match the specified transceiver ID (and wasn’t a Sent-To-All), and the message has been rejected. */
    #define ST_CST_XCVR_CSUM_ERROR      	0x32        	/*!< Returned if received acoustic message’s checksum was invalid, and the message has been rejected. */
    #define ST_CST_XCVR_LENGTH_ERROR    	0x33        	/*!< Returned if an error occurred with message framing, meaning the end of the message has not been received within the expected time. */
    #define ST_CST_XCVR_RESP_TIMEOUT    	0x34        	/*!< Returned if the transceiver has sent a request message to a beacon, but no response has been returned within the allotted waiting period. */
    #define ST_CST_XCVR_RESP_ERROR      	0x35        	/*!< Returned if the transceiver has send a request message to a beacon, but an error occurred while receiving the response. */
    #define ST_CST_XCVR_RESP_WRONG      	0x36        	/*!< Returned if the transceiver has sent a request message to a beacon, but received an unexpected response from another beacon while waiting. */
    #define ST_CST_XCVR_PLOAD_ERROR     	0x37        	/*!< Returned by protocol payload decoders, if the payload can’t be parsed correctly. */
    #define ST_CST_XCVR_STATE_STOPPED   	0x3A        	/*!< Indicates the transceiver is in a stopped state. */
    #define ST_CST_XCVR_STATE_IDLE      	0x3B        	/*!< Indicates the transceiver is in an idle state waiting for reception or transmission to start. */
    #define ST_CST_XCVR_STATE_TX        	0x3C        	/*!< Indicates the transceiver is in a transmitting states. */
    #define ST_CST_XCVR_STATE_REQ       	0x3D        	/*!< Indicates the transceiver is in a requesting state, having transmitted a message and is waiting for a response to be received. */
    #define ST_CST_XCVR_STATE_RX        	0x3E        	/*!< Indicates the transceiver is in a receiving state. */
    #define ST_CST_XCVR_STATE_RESP      	0x3F        	/*!< Indicates the transceiver is in a responding state, where a message is being composed and the “response time” period is being observed. */
    //DEX Protocol Status Codes
    #define ST_CST_DEX_SOCKET_ERROR     	0x70        	/*!< Returned by the DEX protocol handler if an error occurred trying to open, close or access a specified socket ID. */
    #define ST_CST_DEX_RX_SYNC          	0x71        	/*!< Returned by the DEX protocol handler when receiver synchronisation has occurred with the socket master and data transfer is ready to commence. */
    #define ST_CST_DEX_RX_DATA          	0x72        	/*!< Returned by the DEX protocol handler when data has been received through a socket. */
    #define ST_CST_DEX_RX_SEQ_ERROR     	0x73        	/*!< Returned by the DEX protocol handler when data transfer synchronisation has been lost with the socket master. */
    #define ST_CST_DEX_RX_MSG_ERROR     	0x74        	/*!< Returned by the DEX protocol handler to indicate an unexpected acoustic message type with the DEX protocol has been received and cannot be processed. */
    #define ST_CST_DEX_REQ_ERROR        	0x75        	/*!< Returned by the DEX protocol handler to indicate a error has occurred while responding to a request (i.e. lack of data). */
    #define ST_CST_DEX_RESP_TMO_ERROR   	0x76        	/*!< Returned by the DEX protocol handler to indicate a timeout has occurred while waiting for a response back from a remote beacon with requested data. */
    #define ST_CST_DEX_RESP_MSG_ERROR   	0x77        	/*!< Returned by the DEX protocol handler to indicate an error has occurred while receiving response back from a remote beacon. */
    #define ST_CST_DEX_RESP_REMOTE_ERROR 	0x78     		/*!< Returned by the DEX protocol handler to indicate the remote beacon has encountered an error and cannot return the requested data or perform the required operation. */
#endif

//------------------------------------------------------------------------------
//Enumaration that specifies how status mode messages are generated
#ifdef __cplusplus
    enum ESeatracStatusMode : uint8 {
        ST_STATUS_MODE_MANUAL = 0,
        ST_STATUS_MODE_1Hz = 1,
        ST_STATUS_MODE_2Hz5 = 2,
        ST_STATUS_MODE_5Hz = 3,
        ST_STATUS_MODE_10Hz = 4,
    };
#else
    typedef uint8 ESeatracStatusMode;

    #define ST_STATUS_MODE_MANUAL     		0
    #define ST_STATUS_MODE_1Hz          	1
    #define ST_STATUS_MODE_2Hz5         	2
    #define ST_STATUS_MODE_5Hz          	3
    #define ST_STATUS_MODE_10Hz         	4
#endif

//------------------------------------------------------------------------------
//Enumeration that specifies the status of completed command executions
#ifdef __cplusplus
    enum ESeatracExecuteStatus : uint8 {
        ST_EXE_OK = 0,
        ST_EXE_FAIL = 1,
        ST_EXE_UNHANDLED = 2,
        ST_EXE_PARAM_MISSING = 3,
        ST_EXE_PARAM_INVALID = 4,
    };
#else
    typedef uint8 ESeatracExecuteStatus;

    #define ST_EXE_OK            			0
    #define ST_EXE_FAIL            			1
    #define ST_EXE_UNHANDLED            	2
    #define ST_EXE_PARAM_MISSING        	3
    #define ST_EXE_PARAM_INVALID        	4
#endif

//------------------------------------------------------------------------------
//Enumeration specifying Baud Rate values used by the Seatrac Beacon (not the Windows serial port!)
#ifdef __cplusplus
    enum ESeatracBeaconBaud : uint8 {
        ST_BEACON_BAUD_4800 = 0x07,
        ST_BEACON_BAUD_9600 = 0x08,
        ST_BEACON_BAUD_14400 = 0x09,
        ST_BEACON_BAUD_19200 = 0x0A,
        ST_BEACON_BAUD_38400 = 0x0B,
        ST_BEACON_BAUD_57600 = 0x0C,
        ST_BEACON_BAUD_115200 = 0x0D,
        ST_BEACON_BAUD_UNKNOWN = 0xFF,
    };
#else
    typedef uint8 ESeatracBeaconBaud;

    #define ST_BEACON_BAUD_4800             0x07
    #define ST_BEACON_BAUD_9600             0x08
    #define ST_BEACON_BAUD_14400            0x09
    #define ST_BEACON_BAUD_19200            0x0A
    #define ST_BEACON_BAUD_38400            0x0B
    #define ST_BEACON_BAUD_57600            0x0C
    #define ST_BEACON_BAUD_115200           0x0D
    #define ST_BEACON_BAUD_UNKNOWN          0xFF
#endif

//------------------------------------------------------------------------------
//Enumeration sepecifying the type of firmware running
#ifdef __cplusplus
    enum ESeatracAppType : uint8 {
        ST_APPTYPE_BOOT = 0,
        ST_APPTYPE_MAIN = 1,
        ST_APPTYPE_UNKNOWN = 0xFF,
    };
#else
    typedef uint8 ESeatracAppType;

    #define ST_APPTYPE_BOOT           		0
    #define ST_APPTYPE_MAIN            		1
    #define ST_APPTYPE_UNKNOWN          	0xFF
#endif

//------------------------------------------------------------------------------
//Enumeration sepecifying the type of the beacon
#ifdef __cplusplus
    enum ESeatracBeaconType : uint8 {
        ST_BEACONTYPE_UNKNOWN = 0,
        ST_BEACONTYPE_X010 = 1,
        ST_BEACONTYPE_X110 = 2,
        ST_BEACONTYPE_X150 = 3,
    };
#else
    typedef uint8 ESeatracBeaconType;

    #define ST_BEACONTYPE_UNKNOWN     		0
    #define ST_BEACONTYPE_X010          	1
    #define ST_BEACONTYPE_X110          	2
    #define ST_BEACONTYPE_X150          	3
#endif

//------------------------------------------------------------------------------
//Enumeration specifying types of diagnostic messages
#ifdef __cplusplus
    enum ESeatracDiagMsgType : uint8 {
        ST_DIAG_TYPE_NONE = 0,
        ST_DIAG_TYPE_ACTION = 1,
        ST_DIAG_TYPE_DECODE = 2,
        ST_DIAG_TYPE_ENCODE = 3,
        ST_DIAG_TYPE_EXECUTE = 4,
        ST_DIAG_TYPE_PROCESSING = 5,
        ST_DIAG_TYPE_EXECUTED = 6,
    };
#else
    typedef uint8 ESeatracDiagMsgAction;

    #define ST_DIAG_TYPE_NONE               0
    #define ST_DIAG_TYPE_ACTION             1
    #define ST_DIAG_TYPE_DECODE             2
    #define ST_DIAG_TYPE_ENCODE             3
    #define ST_DIAG_TYPE_EXECUTE            4
    #define ST_DIAG_TYPE_PROCESSING         5
    #define ST_DIAG_TYPE_EXECUTED           6
#endif

//------------------------------------------------------------------------------
//Enumeration specifying status of diagnostic messages
#ifdef __cplusplus
    enum ESeatracDiagMsgStatus : uint8 {
        ST_DIAG_STATUS_INFO = 0x00,
        ST_DIAG_STATUS_WARNING = 0x04,
        ST_DIAG_STATUS_ERROR = 0x08,
    };
#else
    typedef uint8 ESeatracDiagMsgStatus;

    #define ST_DIAG_STATUS_INFO             0x00
    #define ST_DIAG_STATUS_WARNING          0x04
    #define ST_DIAG_STATUS_ERROR            0x08
#endif

//------------------------------------------------------------------------------
//Enumeration specifying calibration actions
#ifdef __cplusplus
    enum ESeatracCalAction : uint8 {
        ST_CAL_ACC_DEFAULTS = 0x00,
        ST_CAL_ACC_RESET = 0x01,
        ST_CAL_ACC_CALC = 0x02,
        ST_CAL_MAG_DEFAULTS = 0x03,
        ST_CAL_MAG_RESET = 0x04,
        ST_CAL_MAG_CALC = 0x05,
        ST_CAL_PRES_OFFSET_RESET = 0x06,
        ST_CAL_PRES_OFFSET_CALC = 0x07,
    };
#else
    typedef uint8 ESeatracCalAction;

    #define ST_CAL_ACC_DEFAULTS             0x00
    #define ST_CAL_ACC_RESET                0x01
    #define ST_CAL_ACC_CALC                 0x02
    #define ST_CAL_MAG_DEFAULTS             0x03
    #define ST_CAL_MAG_RESET                0x04
    #define ST_CAL_MAG_CALC                 0x05
    #define ST_CAL_PRES_OFFSET_RESET        0x06
    #define ST_CAL_PRES_OFFSET_CALC         0x07
#endif

//------------------------------------------------------------------------------
//Enumeration specifying the transmit mode of the transceiver
#ifdef __cplusplus
	enum ESeaTracXcvrTxMsgCtrl : uint8 {
		ST_XCVR_TXMSG_ALLOW_ALL = 0,				/*!< Allow Transmission of all message types */
		ST_XCVR_TXMSG_BLOCK_RESP = 1,				/*!< Block Transmission of Response messages - automatically sent when Request messages are received */
		ST_XCVR_TXMSG_BLOCK_ALL = 3,				/*!< Block Transmission of all message types - for additional stealth/airplane mode functionality if required */		
		ST_XCVR_TXMSG_QUERY = 0xFF					/*!< Don't change, just query the current value from the beacon */
	};
#else
	typedef uint8 ESeaTracXcvrTxMsgCtrl;

	#define ST_XCVR_TXMSG_ALLOW_ALL			0		/*!< Allow Transmission of all message types */
	#define ST_XCVR_TXMSG_BLOCK_RESP		1		/*!< Block Transmission of Response messages - automatically sent when Request messages are received */
	#define ST_XCVR_TXMSG_BLOCK_ALL			3		/*!< Block Transmission of all message types - for additional stealth/airplane mode functionality if required */
	#define ST_XCVR_TXMSG_QUERY				0xFF	/*!< Don't change, just query the current value from the beacon */
#endif

//==============================================================================
// Structures
//==============================================================================
//------------------------------------------------------------------------------
typedef struct {
    uint8 Length;                           				/*!< Values specifying how many bytes in the payload array are valid and in use. Valid values are from 0 (for no payload) to 31. */
    uint8 Data[ST_ACOMSG_PAYLOAD_LEN];      				/*!< Array of bytes that contains the payload of the acoustic message. Only the number of bytes specified in the PAYLOAD_LEN parameter are valid, while the contents of the other locations are undefined. The exact function and definition of the PAYLOAD bytes depends on the type of payload specified in the PAYLOAD_ID parameter. */
} TSeatracAcoPayload, *PSeatracAcoPayload;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracBeaconId DestId;                				/*!< Identifier for the beacon the message is or was the recipient of the message. Valid values are 0-15. */
    ESeatracBeaconId SrcId;                 				/*!< Identifier for the beacon that generated or sent the message. Valid values are 1-15. */
    ESeatracAcoMsgType MsgType;            					/*!< Value that indicates the type of message being sent */
    uint16 MsgDepth;                        				/*!< Value that is only valid when MSG_TYPE is MSG_RESPX (an extended USBL response), and contains the depth sensor reading from the remote beacon. The depth is encoded 0.5m steps, and so should be divided by 2 to obtain a depth in metres. */
    ESeatracAcoPayloadType PayloadType;     				/*!< Value that indicates the type of payload the message contains. */
    TSeatracAcoPayload Payload;             				/*!< The payload of the acoustic message */
} TSeatracAcoMsg, *PSeatracAcoMsg;

//------------------------------------------------------------------------------
typedef struct {
    //Received Message Fields
    ESeatracBeaconId DestId;                				/*!< Identifier for the beacon the message is or was the recipient of the message. Valid values are 0-15. */
    ESeatracBeaconId SrcId;                 				/*!< Identifier for the beacon that generated or sent the message. Valid values are 1-15. */
    ESeatracAcoMsgType Type;                				/*!< The type of acoustic message received to generate this fix. */
    float Rssi;                             				/*!< The Received Signal Strength Indicator value for the received message, encoded in centibels. Values are in deciBels. */
} TSeatracAcoFixMsg, *PSeatracAcoFixMsg;

typedef struct {
    //Local Beacon Fields
    bool AttitudeValid;                     				/*!< True if the local beacon supports attitude information */
    float AttitudeYaw;                      				/*!< The yaw angle (relative to magnetic north) of the local beacon when the fix was computed. Values are in degrees. */
    float AttitudePitch;                    				/*!< The pitch angle of the local beacon when the fix was computed. Values are in degrees. */
    float AttitudeRoll;                     				/*!< The roll angle of the local beacon when the fix was computed. Values are in degrees. */
    bool DepthValid;                        				/*!< True if the local beacon supports a depth sensor */
    float Depth;                            				/*!< The reading from the local beacon depth sensor when the fix was calculated. Values are in degrees. */
    float Vos;                              				/*!< The velocity of sound value used for the computation of the remote beacon’s range based on timing information. Values are in meters-per-second. */
} TSeatracAcoFixLocal, *PSeatracAcoFixLocal;

typedef struct {
    //Range Fields
    bool RangeValid;                       				 	/*!< If this bit is set, it indicates the record contains the Range fields below. */
    uint32 RangeCount;                      				/*!< The number of 16kHz timer intervals that were counted between Request message transmission and Response message reception. */
    double RangeTime;                       				/*!< The time in seconds derived from the RANGE_COUNT value, and with internal timing offsets and compensation applied. Values are in seconds. */
    float RangeDist;                        				/*!< The resolved line-of-sight distance to the remote beacon, based on the RANGE_TIME and VOS values. Values are in metres. */

    //USBL Fields
    bool UsblValid;                         				/*!< If this bit is set, it indicates the record contains the USBL fields below. */
    uint8 UsblChannels;                     				/*!< The number of USBL receiver channels being used to compute the signal angle. Typically this value is either 3 or 4. */
    float UsblRssi[ST_USBL_CHANNELS];       				/*!< An array of the received signal strengths for each of the USBL receiver channels, where “x” is the value defined by the CHANNELS field. Values are deci-Bels */
    float UsblAzimuth;                      				/*!< The incoming signal azimuth angle from 0° to 360°. Values are in degrees. */
    float UsblElevation;                    				/*!< The incoming signal elevation angle from -90° to +90°. Values are in degrees. */
    float UsblFitError;                     				/*!< The fit error value returns a number that indicates the quality of fit (or confidence) of the signal azimuth and elevation values from the timing and phase-angle data available. Smaller values towards 0.0 indicate a better fit, while larger values (increasing above 2-3) indicate poorer fits and larger error tolerances. Values are dimensionless. */

    //Position Fields
    bool PositionValid;                     				/*!< If this bit is set, it indicates the record contains the Position fields below. */
    bool PositionEnhanced;                  				/*!< If set, indicates the Position fix has been computed from an Enhanced USBL return – this means the Depth will be the value from the remote beacons depth sensor rather than computed form the incoming signal angle. */
    bool PositionFilterError;               				/*!< If set, indicates the position filter has identified that the position specified in the fix may be invalid based on the beacons previous position, the define beacons motion limits and the time since last communication. However, the position fields still contain the USBL computed position and it is up to the user if they wish to reject this fix, or use it in some direct or weighted fashion. */
    float PositionEasting;                  				/*!< The Easting distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Value in metres. */
    float PositionNorthing;                 				/*!< The Northing distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Value in metres. */
    float PositionDepth;									/*!< The vertical Depth distance component of the remote beacon from the surface - computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Values are in metres. NB: If the ‘Fix’ has been obtained by a MSG_REQU (Usbl) type request, then this value is computed from the beacon’s attitude and incoming signal angle. If a MSG_REQX (Enhanced) type request has been used, then this value is the remotely transmitted beacon depth sensor value. */
} TSeatracAcoFixInfo, *PSeatracAcoFixInfo;

typedef struct {
    uint8 Flags;                            				/*!< Flags decoded from a received AcoFix structure */
    TSeatracAcoFixMsg Msg;                  				/*!< Struct containing a summary of the state of the message received through the acoustic transceiver. */
    TSeatracAcoFixLocal Local;              				/*!< Struct containing the state of the local beacon when the receive occured */
    TSeatracAcoFixInfo Info;                				/*!< Struct containing the fix of the responding beacon computed from the acoustic reception. */
} TSeatracAcoFix, *PSeatracAcoFix;

//------------------------------------------------------------------------------
typedef struct {
    bool Valid;
    uint16 PartNumber;
    uint8 VersionMajor;
    uint8 VersionMinor;
    uint16 VersionBuild;
    uint32 Checksum;
} TSeatracInfoFw, *PSeatracInfoFw;

typedef struct {
    uint16 PartNumber;
    uint8 PartRev;
    uint32 SerialNumber;
    uint16 FlagsSys;
    uint16 FlagsUser;
} TSeatracInfoHw, *PSeatracInfoHw;

//------------------------------------------------------------------------------
//! Strucutre to hold pressure sensor calibration information
typedef struct {
	uint32 Id;
	uint8 Type;
	float PressureMin;
	float PressureMax;
	uint8 CalDay;
	uint8 CalMonth;
	uint16 CalYear;	
} TSeatracPressureCal, *PSeatracPressureCal;

//------------------------------------------------------------------------------
//! Structure that holds data packets sent between beacons using the NAV protocol.
//! (NB: This is shorted than a standard ACOMSG payload beacuause of protocol control bytes)
typedef struct {
    uint8 Length;                           				/*!< Values specifying how many bytes in the payload array are valid and in use. Valid values are from 0 (for no payload) to 30 (not 31!). */
    uint8 Data[ST_NAV_PAYLOAD_LEN];                         /*!< Array of bytes that contains the payload of the acoustic message. Only the number of bytes specified in the LENGTH parameter are valid, while the contents of the other locations are undefined. The exact function and definition of the data is left to the application developer. */
} TSeatracNavPacket, *PSeatracNavPacket;

//------------------------------------------------------------------------------
//! Structure that holds the results of a Navigation Query from a remote beacon
typedef struct {
    uint8 Flags;

    bool DepthValid;
    float DepthRemote;

    bool SupplyValid;
    float SupplyRemote;

    bool TempValid;
    float TempRemote;

    bool AttitudeValid;
    float AttitudeYawRemote;
    float AttitudePitchRemote;
    float AttitudeRollRemote;
} TSeatracNavQuery, *PSeatracNavQuery;

//------------------------------------------------------------------------------
//! Structure that is returned by an XcvrAnalyse command
typedef struct {
    int16 AdcMean;                          				/*!< The average reading seen by the receiving analogue-to-digital converter. */
    uint16 AdcPkPk;                         				/*!< The peak-to-peak reading seen by the receiving analogue-to-digital converter. */
    uint32 AdcRms;                          				/*!< The RMS reading seen by the receiving analogue-to-digital converter. */

    float RxLevelPkPk;                      				/*!< The peak-to-peak noise level observed on the receiver, specified in decibels. This is useful to identify is any short bursts of interference are present during the analysis period. */
    float RxLevelRms;                       				/*!< The RMS noise level observed on the receiver, specified in decibels. This is useful to determine what the general background ambient noise level is. Higher values indicate it is hardware for the receiver to detect valid beacon signals over the noise. */
} TSeatracNoise, *PSeatracNoise;

//------------------------------------------------------------------------------
//! Structure that is returned when an XcvrUsbl diagnostic message is received
typedef struct {
    float XcorSignalPeak;
    float XcorThreshold;
    uint16 XcorCrossingPoint;
    float XcorCrossingMagnitude;
    uint16 XcorDetect;
    uint16 XcorLength;
    float XcorData[ST_USBL_XCOR_DATA_MAX];
    uint8 Channels;
    float ChannelRssi[ST_USBL_CHANNELS];
    uint8 Baselines;
    float BaselinePhaseAngle[ST_USBL_BASELINES];
    float SignalAzimuth;
    float SignalElevation;
    float SignalFitError;
	ESeatracBeaconId DestId;                				/*!< Identifier for the beacon the message is for. Valid values are 0-15. */
    ESeatracBeaconId SrcId;                 				/*!< Identifier for the beacon that sent the message. Valid values are 1-15. */
} TSeatracUsbl, *PSeatracUsbl;

//------------------------------------------------------------------------------
typedef struct {
    int16 AccMinX;
    int16 AccMaxX;
    int16 AccMinY;
    int16 AccMaxY;
    int16 AccMinZ;
    int16 AccMaxZ;

    bool MagValid;
    float MagHardX;
    float MagHardY;
    float MagHardZ;
    float MagSoftX;
    float MagSoftY;
    float MagSoftZ;
    float MagField;
    float MagError;

    int16 GyroOffsetX;
    int16 GyroOffsetY;
    int16 GyroOffsetZ;
} TSeatracAhrsCal, *PSeatracAhrsCal;

//------------------------------------------------------------------------------
typedef struct {
    uint8 Flags;
    double Timestamp;

    bool EnvValid;
    float EnvSupply;
    float EnvTemp;
    float EnvPressure;
    float EnvDepth;
    float EnvVos;

    bool AttitudeValid;
    float AttitudeYaw;
    float AttitudePitch;
    float AttitudeRoll;

    bool MagValid;
    uint8 MagCalBuf;
    bool MagCalValid;
    uint32 MagCalAge;
    uint8 MagCalFit;

    bool AccValid;
    int16 AccLimMinX;
    int16 AccLimMaxX;
    int16 AccLimMinY;
    int16 AccLimMaxY;
    int16 AccLimMinZ;
    int16 AccLimMaxZ;

    bool AhrsRawValid;
    int16 AhrsRawAccX;
    int16 AhrsRawAccY;
    int16 AhrsRawAccZ;
    int16 AhrsRawMagX;
    int16 AhrsRawMagY;
    int16 AhrsRawMagZ;
    int16 AhrsRawGyroX;
    int16 AhrsRawGyroY;
    int16 AhrsRawGyroZ;

    bool AhrsCompValid;
    float AhrsCompAccX;
    float AhrsCompAccY;
    float AhrsCompAccZ;
    float AhrsCompMagX;
    float AhrsCompMagY;
    float AhrsCompMagZ;
    float AhrsCompGyroX;
    float AhrsCompGyroY;
    float AhrsCompGyroZ;
} TSeatracStatus, *PSeatracStatus;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracStatusMode StatusMode;
    bool StatusOutputEnv;
    bool StatusOutputAttitude;
    bool StatusOutputMagCal;
    bool StatusOutputAccCal;
    bool StatusOutputAhrsRaw;
    bool StatusOutputAhrsComp;

    uint32 UartMainBaud;
    uint32 UartAuxBaud;

    bool EnvAutoPressureOffset;
    bool EnvAutoCalcVos;
    float EnvPressureOffset;
    float EnvSalinity;
    float EnvVos;

    bool AhrsAutoCalMag;
    TSeatracAhrsCal AhrsCal;
    float AhrsYawOffset;
    float AhrsPitchOffset;
    float AhrsRollOffset;

    bool XcvrOutputDiagMsgs;
    bool XcvrOutputFixMsgs;
    bool XcvrOutputUsblMsgs;
    bool XcvrEnablePosFilter;
    bool XcvrUsblUseAhrs;	
    ESeatracBeaconId XcvrBeaconId;
    uint16 XcvrRangeTimeout;                				/*!< The maximum velocity limit (in metres per second) that the position filter expects to see a beacon move at. Position Fix outputs for Beacons that have moved faster than this in the time between pings will be marked as a position error. Values are encoded in metres. Valid values are in the range 100m to 3000m. */
    uint16 XcvrResponseTime;                				/*!< The response turnaround time specifies how long the beacon will wait between receiving a request message and starting transmission of the response message. All beacons communicating acoustically within the same network must use the same value otherwise range errors will be observed. Values are encoded in milliseconds. Valid values are in the range 10ms to 1000ms - the default is 10ms. */
    float XcvrFixedYaw;
    float XcvrFixedPitch;
    float XcvrFixedRoll;
    uint8 XcvrPosFilterVelocity;            				/*!< The maximum velocity limit (in metres per second) that the position filter expects to see a beacon move at. Position Fix outputs for Beacons that have moved faster than this in the time between pings will be marked as a position error. */
    uint8 XcvrPosFilterAngularRate;         				/*!< For beacons that are further away, azimuth errors start to come into play. This value defines the angular limits that beacons can move (or position jitter) within without being marked as an error. Vales are specified in degrees, and typically this value is 10 degrees. */
    uint8 XcvrPosFilterTimeout;             				/*!< This timeout limit specified in seconds that maximum time that a beacon is not communicated with before its position filter is reset, allowing its next position (what ever that may be) to be marked as valid. */
	ESeaTracXcvrTxMsgCtrl XcvrTxMsgCtrl;					/*!< The Transceiver Message Control Setting */
} TSeatracSettings, *PSeatracSettings;

//==============================================================================
// Callback Parameter Structs (passed by Events)
//==============================================================================
// To allow thread safety and passing or event parameter structs through
// operating system event/messaging stacks, the param structs must be
// self contained (i.e. no pointers or classes) to allow copying by
// dereferenced pointer assignment (TxxxParams param = *param_ptr);
//==============================================================================
typedef struct {
    ESeatracCmdStatus CmdStatus;
} TSeatracCmdStatusParams, *PSeatracCmdStatusParams;

//------------------------------------------------------------------------------
//! Define a structure that is passed to callbacks when a command line is decoded
typedef struct {
    char Buffer[1024];
    uint32 Length;
    bool HasMsg;
} TSeatracCmdDecodeLineParams, *PSeatracCmdDecodeLineParams;

//------------------------------------------------------------------------------
//! Define a structure that contains parameters for the OnEncode callback
typedef struct {
    char Buffer[1024];
    uint32 Length;
    bool Success;
} TSeatracCmdEncodeParams, *PSeatracCmdEncodeParams;

//------------------------------------------------------------------------------
//! Define a structure that contains the parameters for the OnCfgBeaconGet & OnCfgBeaconSet events
typedef struct {
    ESeatracCmdStatus CmdStatus;
    uint16 SerialNumber;
} TSeatracCfgBeaconAckParams, *PSeatracCfgBeaconAckParams;

//------------------------------------------------------------------------------
//! Define a structure that contains the parameters for the OnCfgBeaconResp event
typedef struct {
	ESeatracCmdStatus CmdStatus;
    uint16 SerialNumber;
	ESeatracBeaconId BeaconId;
	uint16 RangeTimeout;
	uint16 ResponseTurnaround;
} TSeatracCfgBeaconRespParams, *PSeatracCfgBeaconRespParams;

//------------------------------------------------------------------------------
//Same as PingSend and EchoSend
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;
} TSeatracDatSendParams, *PSeatracDatSendParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    bool Ack;                               				/*!< True if the receive is a response from a remote beacon, that may contain retreived queued data */
    bool LocalFlag;                                         /*!< True if the data packet is intended for the local beacon, false if its been 'sniffed' between other beacons */
    TSeatracAcoFix Fix;                     				/*!< A request structure containing information received from the requesting beacon, and the local beacons status. */
    TSeatracAcoPayload Payload;                              /*!< The payload of the Data message received */
} TSeatracDatReceiveParams, *PSeatracDatReceiveParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;                                /*!< The ID of the beacon data was queued for */
    uint8 Length;                                           /*!< The length of the packet of data that was queued for the specified beacon */
} TSeatracDatQueueSetParams, *PSeatracDatQueueSetParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;                                /*!< The ID of the beacon data was cleared for */
} TSeatracDatQueueClrParams, *PSeatracDatQueueClrParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    uint8 Length[ST_BEACONS];                               /*!< Array of lengths of data queued for each destination beacon */
} TSeatracDatQueueStatusParams, *PSeatracDatQueueStatusParams;

//------------------------------------------------------------------------------
//! Structure that contains paramters passed to a DiagLog event
typedef struct {
    uint32 Id;
    double Timestamp;               //In seconds since object creation
    ESeatracDiagMsgStatus Status;
    char StatusStr[16];
    char NameStr[32];
    ESeatracDiagMsgType Type;
    char TypeStr[16];
    bool CmdIdValid;
    ESeatracCmdId CmdId;
    char CmdIdStr[32];
    char ParamsStr[1024];
} TSeatracDiagLogParams, *PSeatracDiagLogParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;
} TSeatracEchoSendParams, *PSeatracEchoSendParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoFix Request;                 				/*!< A request structure containing information received from the requesting beacon, and the local beacons status. */
    TSeatracAcoPayload Payload;             				/*!< The payload of the Echo message received */
} TSeatracEchoRequestParams, *PSeatracEchoRequestParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    TSeatracAcoFix Response;                				/*!< A response structure containing information received back from the responding beacon, and the local beacons status - or if an error occured waiting for a response. */
    TSeatracAcoPayload Payload;             				/*!< The payload of the Echo message received */
} TSeatracEchoResponseParams, *PSeatracEchoResponseParams;

//------------------------------------------------------------------------------
//! typedef struct that contains paramters passed to an ExecuteDone event
typedef struct {
    ESeatracCmdId Id;                       				/*!< The ID code of the message that was executed */
    ESeatracExecuteStatus ExeStatus;        				/*!< The status code when execution was completed */
} TSeatracExecuteDoneParams, *PSeatracExecuteDoneParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;                                /*!< The ID of the beacon the query was sent to */
} TSeatracNavQuerySendParams, *PSeatracNavQuerySendParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoFix Request;                 				/*!< A request structure containing information received from the requesting beacon, and the local beacons status. */
    bool LocalFlag;                                         /*!< True if the packet is intended for the local beacon, false if its been 'sniffed' between other beacons */
    uint8 QueryFlags;                                       /*!< Flags specify what information has been queried */
    TSeatracNavPacket Payload;                               /*!< Any data that was sent in the request */
} TSeatracNavQueryRequestParams, *PSeatracNavQueryRequestParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    TSeatracAcoFix Response;                				/*!< A response structure containing information received back from the responding beacon, and the local beacons status - or if an error occured waiting for a response. */
    bool LocalFlag;                                         /*!< True if the packet is intended for the local beacon, false if its been 'sniffed' between other beacons */
    uint8 QueryFlags;                                       /*!< Flags specify what information has been queried */
    TSeatracNavQuery Query;                                 /*!< Query parameter informaiton */
    TSeatracNavPacket Payload;                               /*!< Any queried data that was returned in the response, if the QRY_DATA bit was set in the sent (requesting) NQV_QUERY_SEND message */
} TSeatracNavQueryResponseParams, *PSeatracNavQueryResponseParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;                                /*! The Id of the beacon queue that was set (or queried) */
    uint8 Length;                                           /*! The number of bytes queued pending transmission for the specified beacon */
} TSeatracNavQueueSetParams ,*PSeatracNavQueueSetParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;                                /*! The Id of the beacon queue that was cleared */
} TSeatracNavQueueClrParams ,*PSeatracNavQueueClrParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    uint8 Length[ST_BEACONS];                               /*!< Array of lengths of data queued for each destination beacon */
} TSeatracNavQueueStatusParams ,*PSeatracNavQueueStatusParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId BeaconId;                              /*!< NAV_STATUS messages are sent to all beacons, so this is the ID of the beacon the status information in the message related to */
} TSeatracNavStatusSendParams ,*PSeatracNavStatusSendParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoFix Fix;                                     /*!< A request structure containing information received from the requesting beacon, and the local beacons status. */
    ESeatracBeaconId BeaconId;                              /*!< NAV_STATUS messages are sent to all beacons, so this is the ID of the beacon the status information in the message related to */
    bool LocalFlag;                                         /*!< True if the packet is intended for the local beacon, false if its been 'sniffed' between other beacons */
    TSeatracNavPacket Payload;                               /*!< Any queried data that was returned in the response, if the QRY_DATA bit was set in the sent (requesting) NQV_QUERY_SEND message */
} TSeatracNavStatusReceiveParams ,*PSeatracNavStatusReceiveParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    ESeatracBeaconId DestId;
} TSeatracPingSendParams, *PSeatracPingSendParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoFix Request;                 				/*!< A request structure containing information received from the requesting beacon, and the local beacons status. */
} TSeatracPingRequestParams, *PSeatracPingRequestParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    TSeatracAcoFix Response;                				/*!< A response structure containing information received back from the responding beacon, and the local beacons status - or if an error occured waiting for a response. */
} TSeatracPingResponseParams, *PSeatracPingResponseParams;

//------------------------------------------------------------------------------
typedef struct {
    bool Error;
} TSeatracSerialCloseParams, *PSeatracSerialCloseParams;

//------------------------------------------------------------------------------
typedef struct {
    bool State;
} TSeatracSerialTimeoutParams, *PSeatracSerialTimeoutParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracSettings Settings;
} TSeatracSettingsGetParams, *PSeatracSettingsGetParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracStatus Status;
} TSeatracStatusParams, *PSeatracStatusParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracStatusMode Mode;
    uint8 Flags;
} TSeatracStatusCfgGetParams, *PSeatracStatusCfgGetParams;

//------------------------------------------------------------------------------
typedef struct {
    uint32 Seconds;
} TSeatracSysAliveParams, *PSeatracSysAliveParams;

//------------------------------------------------------------------------------
typedef struct {
    uint32 Seconds;                     /*!< Number of seconds the beacon has been powered on for */
    ESeatracAppType AppType;            /*!< The firmware mode the beacon is running, bootloader or main application */
    ESeatracBeaconType BeaconType;      /*!< The type of Seatrac beacon */
    TSeatracInfoHw Hardware;            /*!< Details on the beacon hardware */
    TSeatracInfoFw BootApp;             /*!< Firmware details for the bootloader application in the beacon */
    TSeatracInfoFw MainApp;             /*!< Firmware details for the main application in the beacon */    
	TSeatracPressureCal PressureCal;	/*!< Pressure sensor calibration information - only populate in AppType is MAIN */
	bool MainSupported;
} TSeatracSysInfoParams, *PSeatracSysInfoParams;

//------------------------------------------------------------------------------
typedef struct {
    ESeatracCmdStatus CmdStatus;
    TSeatracNoise Noise;
} TSeatracXcvrAnalyseParams, *PSeatracXcvrAnalyseParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoFix AcoFix;
} TSeatracXcvrFixParams, *PSeatracXcvrFixParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoMsg AcoMsg;
    TSeatracAcoFix AcoFix;
} TSeatracXcvrMsgFixParams, *PSeatracXcvrMsgFixParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracAcoMsg AcoMsg;
} TSeatracXcvrMsgParams, *PSeatracXcvrMsgParams;

//------------------------------------------------------------------------------
//Could this be merfed with the XcvrFixParams struct, if CmdStatus=OK was added into that
typedef struct {
    ESeatracCmdStatus CmdStatus;
    TSeatracAcoFix AcoFix;
} TSeatracXcvrRxErrorParams, *PSeatracXcvrRxErrorParams;

//------------------------------------------------------------------------------
typedef struct  {
    ESeatracCmdStatus CmdStatus;
    TSeatracAcoFix AcoFix;
} TSeatracXcvrStatusParams, *PSeatracXcvrStatusParams;

//------------------------------------------------------------------------------
typedef struct {
	ESeatracCmdStatus CmdStatus;
	ESeaTracXcvrTxMsgCtrl TxMsgCtrl;
} TSeatracXcvrTxMsgCtrlSetParams, *PSeatracXcvrTxMsgCtrlSetParams;

//------------------------------------------------------------------------------
typedef struct {
    TSeatracUsbl Usbl;
} TSeatracXcvrUsblParams, *PSeatracXcvrUsblParams;

//==============================================================================
//#pragma pack(pop)
#endif
