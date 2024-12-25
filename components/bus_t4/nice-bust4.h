/*
  Nice BusT4
  Data exchange via UART at 19200 baud, 8n1
  Before a data packet, a break signal with a duration of 519us (10 bits) is sent
  The content of the packet that could be understood is described in the structure packet_cmd_body_t

  For Oview, the address always adds 80.
  The gate controller address remains unchanged.

Connection:

BusT4                       ESP8266

Device wall        Rx Tx GND
9  7  5  3  1
10 8  6  4  2
Cable slot:
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V

From the manual nice_dmbm_integration_protocol.pdf:

• ADR: This is the NICE network address where the devices you want to control are located. It can range from 1 to 63 (from 1 to 3F).
This value must be in HEX. If the recipient is the DIN-BAR integration module, this value is 0 (adr = 0); if the recipient
is a smart motor, this value is 1 (adr = 1).
• EPT: This is the address of the Nice motor within the network ADR. It can range from 1 to 127. This value must be in HEX.
• CMD: This is the command you want to send to the destination (ADR, EPT).
• PRF: Command for setting the profile.
• FNC: This is the function you want to send to the destination (ADR, EPT).
• EVT: This is the event sent to the destination (ADR, EPT).
*/

#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // For adding Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // For parsing strings with built-in tools
#include <queue>                               // For working with queues

#if defined(ESP32)
#include "driver/uart.h"
#endif

namespace esphome {
namespace bus_t4 {

/* For short references to class members */
using namespace esphome::cover;

static const uint32_t BAUD_BREAK = 9200; /* Baud rate for the long pulse before a packet */
static const uint32_t BAUD_WORK = 19200; /* Working baud rate */
static const uint8_t START_CODE = 0x55; /* Start byte of the packet */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // The position value below which the gate is considered fully closed
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Interval for updating the current gate position in milliseconds

/* ESP network settings
  The series can take values from 0 to 63, default is 0
  OVIEW addresses start from 8

  When networking multiple drives with OXI, specify different series for each drive.
  In this case, the OXI series must match that of the drive it controls.
*/

/* Message types in packets
   Currently, we are only interested in CMD and INF
   Others have not been deeply studied, and their numbers are unverified
   6th byte of CMD and INF packets
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* Verified number, sending commands to the automation */
  INF = 0x08,  /* Returns or sets information about the device */
};

/* Command menu in the Oview hierarchy
   9th byte of CMD packets
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};

/* Used in STA responses */
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // Operation completed by timeout
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // Partial opening
};

/* Errors */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // No errors
  FD = 0xFD,    // Command not available for this device
};

// Motor types
enum motor_type  : uint8_t {
  SLIDING = 0x01,
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // Up-and-over gates
};

//  9th byte
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* Packet for/from all */
  FOR_CU = 0x04,  /* Packet for/from the control unit */
  FOR_OXI = 0x0A,  /* Packet for/from the OXI receiver */
};

// 10th byte of GET/SET EVT packets, only the RUN value was observed in CMD packets
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Request motor type */
  INF_STATUS     = 0x01, // Gate status (Open/Closed/Stopped)
  WHO	         = 0x04,  /* Who is on the network? */
  MAC            = 0x07,    // MAC address
  MAN            = 0x08,   // Manufacturer
  PRD            = 0x09,   // Product
  INF_SUPPORT    = 0x10, //  Available INF commands
  HWR            = 0x0a,   // Hardware version
  FRM            = 0x0b,   // Firmware version
  DSC            = 0x0c,   // Description
  CUR_POS        = 0x11,  // Current position of the automation
  MAX_OPN        = 0x12,   // Maximum possible opening position (encoder)
  POS_MAX        = 0x18,   // Maximum opening position (encoder)
  POS_MIN        = 0x19,   // Minimum closing position (encoder)
  INF_P_OPN1     = 0x21, // Partial opening 1
  INF_P_OPN2     = 0x22, // Partial opening 2
  INF_P_OPN3     = 0x23, // Partial opening 3
  INF_SLOW_OPN   = 0x24, // Slowdown when opening
  INF_SLOW_CLS   = 0x25, // Slowdown when closing
  OPN_OFFSET     = 0x28, /* Opening offset */
  CLS_OFFSET     = 0x29, /* Closing offset */
  OPN_DIS        = 0x2a, /* Open discharge */
  CLS_DIS        = 0x2b, /* Close discharge */
  REV_TIME       = 0x31, /* Brief inversion value */
  OPN_PWR        = 0x4A,    /* Force adjustment - opening force */
  CLS_PWR        = 0x4B,    /* Force adjustment - closing force */
  SPEED_OPN      = 0x42,    /* Speed adjustment - opening speed */
  SPEED_CLS      = 0x43,    /* Speed adjustment - closing speed */
  SPEED_SLW_OPN  = 0x45,    /* Speed adjustment - slow opening speed */
  SPEED_SLW_CLS  = 0x46,    /* Speed adjustment - slow closing speed */
  OUT1           = 0x51,  /* Output configuration */
  OUT2           = 0x52,  /* Output configuration */
  LOCK_TIME      = 0x5A,  /* Lock operation time */
  S_CUP_TIME     = 0x5C,  /* Suction cup operation time */
  LAMP_TIME      = 0x5B,  /* Courtesy light operation time */
  COMM_SBS       = 0x61,  /* Step-by-step */
  COMM_POPN      = 0x62,  /* Open partially */
  COMM_OPN       = 0x63,  /* Open */
  COMM_CLS       = 0x64,  /* Close */
  COMM_STP       = 0x65,  /* STOP */
  COMM_PHOTO     = 0x68,  /* Photo */
  COMM_PHOTO2    = 0x69,  /* Photo 2 */
  COMM_PHOTO3    = 0x6A,  /* Photo 3 */
  COMM_OPN_STP   = 0x6B,  /* Stop while opening */
  COMM_CLS_STP   = 0x6C,  /* Stop while closing */
  IN1            = 0x71,  /* Input configuration */
  IN2            = 0x72,  /* Input configuration */
  IN3            = 0x73,  /* Input configuration */
  IN4            = 0x74,  /* Input configuration */
  COMM_LET_OPN   = 0x78,  /* Interference with opening */
  COMM_LET_CLS   = 0x79,  /* Interference with closing */
  AUTOCLS        = 0x80,    /* Auto-closing */
  P_TIME         = 0x81,    /* Pause time */
  PH_CLS_ON      = 0x84,    /* Photo-close active */
  PH_CLS_VAR     = 0x86,    /* Photo-close mode */
  PH_CLS_TIME    = 0x85,    /* Photo-close waiting time */
  ALW_CLS_ON     = 0x88,    /* Always close active */
  ALW_CLS_VAR    = 0x8A,    /* Always close mode */
  ALW_CLS_TIME   = 0x89,    /* Always close waiting time */
  STAND_BY_ACT   = 0x8c,    /* Standby mode active */
  WAIT_TIME      = 0x8d,    /* Standby wait time */
  STAND_BY_MODE  = 0x8e,    /* Standby mode */
  START_ON       = 0x90,    /* Startup active */
  START_TIME     = 0x91,    /* Startup time */
  SLOW_ON        = 0xA2,    /* Slow mode */
  DIS_VAL        = 0xA4,    /* Disable value */
  BLINK_ON       = 0x94,    /* Pre-blink active */
  BLINK_OPN_TIME = 0x95,    /* Pre-blink time while opening */
  BLINK_CLS_TIME = 0x99,    /* Pre-blink time while closing */
  OP_BLOCK       = 0x9a,    /* Operator block */
  KEY_LOCK       = 0x9c,    /* Key lock */
  T_VAL          = 0xB1,    /* Alarm threshold value */
  P_COUNT        = 0xB2,    /* Partial count */
  C_MAIN         = 0xB4,    /* Cancel maintenance */
  DIAG_BB        = 0xD0,    /* Bluebus diagnostics */
  INF_IO         = 0xD1,    /* Input-output status */
  DIAG_PAR       = 0xD2,    /* Other diagnostics parameters */
  CUR_MAN        = 0x02,    /* Current maneuver */
  SUBMNU         = 0x04,    /* Submenu */
  STA            = 0xC0,    /* Status in motion */
  MAIN_SET       = 0x80,    /* Main settings */
  RUN            = 0x82,    /* Command execution */
};

/* Specific position handling logic should be integrated below to accommodate proper hex value representation. */
