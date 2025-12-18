/*
  Nice BusT4
  UART data exchange at 19200 8n1
  A break of 519us duration (10 bits) is sent before the data packet.
  The packet contents, as we were able to understand them, are described in the packet_cmd_body_t structure.

 

  For Oview, 80 is always added to the address.
  The gate controller address remains unchanged.


Connection

BusT4                       ESP8266

Device side        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
Cable location
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




From the nice_dmbm_integration_protocol.pdf manual

• ADR: This is the address of the NICE network where the devices you want to control are located. This can be a value from 1 to 63 (1 to 3F).
This value must be in HEX. If the destination is a DIN-BAR integration module, this value is 0 (adr = 0); if the destination is a smart motor, this value is 1 (adr = 1).
• EPT: This is the address of the Nice engine included in the network ADR. This can be a value from 1 to 127. This value must be in HEX format.
• CMD: This is the command you want to send to the destination (ADR, EPT).
• PRF: The profile setup command.
• FNC: This is the function you want to send to the destination (ADR, EPT).
• EVT: This is the event that is sent to the destination (ADR, EPT).



*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // to add Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // parsing strings with built-in tools
#include <queue>                               // to work with the queue



namespace esphome {
namespace bus_t4 {

/* for short addressing of class members */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;


static const int _UART_NO=UART0; /* number uart */
static const int TX_P = 1;         /* pin Tx */
static const uint32_t BAUD_BREAK = 9200; /* baudrate for a long pulse before the burst */
static const uint32_t BAUD_WORK = 19200; /* working baudrate */
static const uint8_t START_CODE = 0x55; /*packet start byte */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // The drive position value in percent, below which the gate is considered fully closed
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Current drive position update interval, ms

/* ESP network settings
  The series can take values ​​from 0 to 63, the default is 0
  OVIEW address starts with 8

  When networking multiple OXI drives, different rows must be specified for each drive.
  In this case, the OXI row must be the same as the drive it controls.
*/






/* Packet message type
  For now we are only interested in CMD and INF
  I didn't study the rest in depth and didn't check the numbers.
  6th byte of CMD and INF packets
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* number checked, sending commands to automation */
//  LSC = 0x02,  /* working with script lists */
//  LST = 0x03,  /* working with automation lists */
//  POS = 0x04,  /* request and change of automation position */
//  GRP = 0x05,  /* sending commands to the automation group with the motor bit mask specified */
//  SCN = 0x06,  /* working with scripts */
//  GRC = 0x07,  /* sending commands to a group of automations created via Nice Screen Configuration Tool */
  INF = 0x08,  /* gets or sets information about the device */
//  LGR = 0x09,  /* working with group lists */
//  CGR = 0x0A,  /* working with categories of groups created via Nice Screen Configuration Tool */
};




/* 
command menu in the oview hierarchy
9th byte of CMD packets
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* used in STA responses*/
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // the maneuver was completed due to a timeout
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // partial opening
};

/* Errors */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // No errors
  FD = 0xFD,    // There is no command for this device.
  };

// Motor types
enum motor_type  : uint8_t {
  SLIDING = 0x01, 
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over lift-and-turn gates
  };

//  ninth byte
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* package for/from everyone */
  FOR_CU = 0x04,  /* package for/from the control unit */
  FOR_OXI = 0x0A,  /* package for/from OXI receiver */
  };
	
// the tenth byte of GET/SET EVT packets; for CMD packets, only the RUN value was encountered
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Request drive type */
  INF_STATUS     = 0x01, //	Gate status (Open/Closed/Stopped)
  WHO	         = 0x04,  /* Who's online? */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // manufacturer.
  PRD            = 0x09,   // product.
  INF_SUPPORT    = 0x10, //  Available INF commands
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // the current conditional position of the automation, DPRO924 then waits for the positions to be set
  MAX_OPN        = 0x12,   // Maximum possible opening by encoder.
  POS_MAX        = 0x18,   // Maximum position (opening) according to encoder
  POS_MIN        = 0x19,   // Minimum position (closing) according to encoder
  INF_P_OPN1     = 0x21, //	Partial opening1 
  INF_P_OPN2     = 0x22, //	Partial opening2
  INF_P_OPN3     = 0x23, //	Partial opening3
  INF_SLOW_OPN   = 0x24, // Slowness in opening
  INF_SLOW_CLS   = 0x25, // Slow closing
  OPN_OFFSET     = 0x28, /* Opening delay open offset */
  CLS_OFFSET     = 0x29, /* Close delay close offset */
  OPN_DIS        = 0x2a, /* Main parameters - Open discharge */
  CLS_DIS        = 0x2b, /* Main parameters - Close discharge */
  REV_TIME       = 0x31, /* Basic parameters - Brief inversion value */
  OPN_PWR        = 0x4A,    /* Main parameters - Force control - Opening force */	  	  	  	  
  CLS_PWR        = 0x4B,    /* Main parameters - Force control - Closing force */  	  	  	  	  
  SPEED_OPN      = 0x42,    /* Main parameters - Speed ​​setting - Opening speed */ 	  	  	  	  	  	  
  SPEED_CLS      = 0x43,    /* Main parameters - Speed ​​setting - Closing speed */
  SPEED_SLW_OPN  = 0x45,    /* Main parameters - Speed ​​setting - Slow opening speed */
  SPEED_SLW_CLS  = 0x46,    /* Main parameters - Speed ​​setting - Slow closing speed */
  OUT1           = 0x51,  /* Setting up outputs */
  OUT2           = 0x52,  /* Setting up outputs */
  LOCK_TIME      = 0x5A,  /* Output settings - Lock operating time */
  S_CUP_TIME     = 0x5C,  /* Output Settings - Suction Cup Time */
  LAMP_TIME      = 0x5B,  /* Output settings - Courtesy light Time */
  COMM_SBS       = 0x61,  /* Setting up commands - Step by step */
  COMM_POPN      = 0x62,  /* Setting up commands - Partial Open */ 	  
  COMM_OPN       = 0x63,  /* Setting up commands - Open */	  	  	  
  COMM_CLS       = 0x64,  /* Setting up commands - Close */	  
  COMM_STP       = 0x65,  /* Setting up commands - STOP */		  
  COMM_PHOTO     = 0x68,  /* Setting up commands - Photo */		  
  COMM_PHOTO2    = 0x69,  /* Setting up commands - Photo2 */
  COMM_PHOTO3    = 0x6A,  /* Setting up commands - Photo3 */
  COMM_OPN_STP   = 0x6B,  /* Setting up commands - STOP when opening */	  
  COMM_CLS_STP   = 0x6C,  /* Setting up commands - STOP when closing */	 
  IN1            = 0x71,  /* Setting up inputs */
  IN2            = 0x72,  /* Setting up inputs */
  IN3            = 0x73,  /* Setting up inputs */
  IN4            = 0x74,  /* Setting up inputs */
  COMM_LET_OPN   = 0x78,  /* Setting up commands - Obstruction of opening */	  	  	  
  COMM_LET_CLS   = 0x79,  /* Setting up commands - Obstacle to closing */	  	  	  	  

  AUTOCLS        = 0x80,    /* Main parameters - Auto-closing */
  P_TIME         = 0x81,    /* Main parameters - Время паузы */
  PH_CLS_ON      = 0x84,    /* Main parameters - Close after Photo - Active */	  
  PH_CLS_VAR     = 0x86,    /* Main parameters - Close after Photo - Mode */	  	  
  PH_CLS_TIME    = 0x85,    /* Main parameters - Close after Photo - Waiting Time */	  	  	  
  ALW_CLS_ON     = 0x88,    /* Main parameters - Always Close - Active */	  	  
  ALW_CLS_VAR    = 0x8A,    /* Main parameters - Always Close - Mode */	  
  ALW_CLS_TIME   = 0x89,    /* Main parameters - Always Close - Timeout */	  	  	  
  STAND_BY_ACT   = 0x8c,    /* Main parameters - Standby Mode - Active ON/OFF */
  WAIT_TIME      = 0x8d,    /* Main parameters - Standby Mode - Standby Time */
  STAND_BY_MODE  = 0x8e,    /* Main parameters - Standby Mode - Mode -  safety = 0x00, bluebus=0x01, all=0x02*/
  START_ON       = 0x90,    /* Main parameters - Start Setting - Active */		  	  
  START_TIME     = 0x91,    /* Main parameters - Start Setting - Start Time */		  	  	  
  SLOW_ON        = 0xA2,    /* Main parameters - Slowing down */	
  DIS_VAL        = 0xA4,    /* Position - Value is invalid disable value */

  BLINK_ON       = 0x94,    /* Main parameters - Flickering - Actively */		  	  	  	  
  BLINK_OPN_TIME = 0x95,    /* Main parameters - Flickering - Time at opening */		  	  	  	  	  
  BLINK_CLS_TIME = 0x99,    /* Main parameters - Flickering - Time at closing */
  OP_BLOCK       = 0x9a,    /* Main parameters - Engine blocking (Operator block)*/
  KEY_LOCK       = 0x9c,    /* Main parameters - Blocking buttons */
  T_VAL          = 0xB1,    /*Alarm threshold value Threshold before servicing in number of maneuvers*/
  P_COUNT        = 0xB2,    /* Partial count Dedicated counter*/
  C_MAIN         = 0xB4,    /* Cancel maintenance Cancellation of service */
  DIAG_BB        = 0xD0,     /*   DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /*	input-output state	*/
  DIAG_PAR       = 0xD2,    /*  DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // Current Maneuver
  SUBMNU  = 0x04,  // Submenu
  STA = 0xC0,   // status in motion
  MAIN_SET = 0x80,   // Main parameters
  RUN = 0x82,   // Command to execute

  };	

	
/* run cmd 11th byte of EVT packets */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* request to change parameters */
  GET = 0x99,   /* request for parameters */
  GET_SUPP_CMD = 0x89, /* get supported commands */
  };


/* The command to be executed.
11th byte of CMD packet
Used in requests and responses */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - Partial opening, wicket mode */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* interface response acknowledging receipt of the command */
  EVT = 0x29, /* interface response sending the requested information */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - Collectively */
  P_OPN5 = 0x0c, /* Partial opening 5 - Priority step by step */
  P_OPN6 = 0x0d, /* Partial opening 6 - Open and lock */
  UNLK_OPN = 0x19, /* Unlock and Open */
  CLS_LOCK = 0x0E, /* Close and lock */
  UNLCK_CLS = 0x1A, /*  Unlock and Close */
  LOCK = 0x0F, /* Block */
  UNLOCK = 0x10, /* Unblock */
  LIGHT_TIMER = 0x11, /* Lighting timer */
  LIGHT_SW = 0x12, /* Lighting on/off */
  HOST_SBS = 0x13, /* Leading SBS */
  HOST_OPN = 0x14, /* Leading Open */
  HOST_CLS = 0x15, /* Leading Close */
  SLAVE_SBS = 0x16, /*  Slave SBS */
  SLAVE_OPN = 0x17, /* Slave Open */
  SLAVE_CLS = 0x18, /* Slave Close */
  AUTO_ON = 0x1B, /* Autoopening Actively */
  AUTO_OFF = 0x1C, /* Autoopening is inactive	  */
  
};
	
	
	
	
	
	
/* Information for a better understanding of the composition of packets in the protocol */
// CMD request packet body
// packets with body size 0x0c=12 bytes
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // packet body size (excluding header and CRC. Total number of bytes minus three), for commands = 0x0c
  uint8_t for_series;           // series to whom the ff package = everyone
  uint8_t for_address;          // Address to whom the ff package is sent = everyone
  uint8_t from_series;           // series from whom is the package
  uint8_t from_address;          // series from whom is the package
  uint8_t mes_type;           // message type, 1 = CMD, 8 = INF
  uint8_t mes_size;              // the number of bytes further minus the two CRC bytes at the end, for commands = 5
  uint8_t crc1;                // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu;                // Command menu. cmd_mnu = 1 for control commands
  uint8_t setup_submnu;            // The submenu, in combination with the command group, determines the type of message being sent.
  uint8_t control_cmd;            // The command to be executed
  uint8_t offset;            //  Response offset. Affects queries like the list of supported commands.
  uint8_t crc2;            // crc2, XOR of the previous four bytes
  uint8_t pct_size2;            // packet body size (excluding header and CRC. Total number of bytes minus three), for commands = 0x0c

};





// RSP response packet body
// packets with body size 0x0e=14 bytes
struct packet_rsp_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // packet body size (excluding header and CRC, total number of bytes minus three), >= 0x0e
  uint8_t to_series;           // series to whom the ff package = everyone
  uint8_t to_address;          // Address to whom the ff package is sent = everyone
  uint8_t from_series;           // series from whom is the package
  uint8_t from_address;          // address from whom the package is
  uint8_t mes_type;           // message type, for these packets always 8 = INF
  uint8_t mes_size;              // the number of bytes further minus the two CRC bytes at the end, for commands = 5
  uint8_t crc1;                // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu;                // Command menu. cmd_mnu = 1 for control commands
  uint8_t sub_inf_cmd;            // From which submenu the command was received. The value is 0x80 less than the original submenu.
  uint8_t sub_run_cmd;            // What command was received? The value is 0x80 greater than the command received.
  uint8_t hb_data;             // data, most significant bit
  uint8_t lb_data;            // data, least significant bit
  uint8_t err;               // Errors
  uint8_t crc2;            // crc2, XOR of the previous four bytes
  uint8_t pct_size2;            // packet body size (excluding header and CRC, total number of bytes minus three), >= 0x0e

};
	
 // EVT data response packet body
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // packet body size (excluding header and CRC, total number of bytes minus three), >= 0x0e
  uint8_t to_series;           // series to whom the ff package = everyone
  uint8_t to_address;          // Address to whom the ff package is sent = everyone
  uint8_t from_series;           // series from whom is the package
  uint8_t from_address;          // address from whom the package is
  uint8_t mes_type;           // address from whom themessage type, for these packets always 8 = INF package is
  uint8_t mes_size;              // address from whothe number of bytes further minus the two CRC bytes at the end, for commands = 5m themessage type, for these packets always 8 = INF package is
  uint8_t crc1;                // CRC1, XOR of the previous six bytes
  uint8_t whose;                // CRC1, XOR of the six previous packets. Options: 00 - general, 04 - drive controller, 0A - receiver OXI bytes
  uint8_t setup_submnu;            // CRC1, XOR of thFrom which submenu the command was received. The value is equal to the original submenu.e six previous packets. Options: 00 - general, 04 - drive controller, 0A - receiver OXI bytes
  uint8_t sub_run_cmd;            // CRC1, XOR ofWhat command are we responding to? The value is 0x80 less than the previously sent command. thFrom which submenu the command was received. The value is equal to the original submenu.e six previous packets. Options: 00 - general, 04 - drive controller, 0A - receiver OXI bytes
  uint8_t next_data;            // Next data block
  uint8_t err;               // Errors
  uint8_t data_blk;            // Data block, may occupy several bytes
  uint8_t crc2;            // Data block, crc2, XOR of all previous bytes up to the ninth (Whose packet)may occupy several bytes
  uint8_t pct_size2;            // Data block, ccrc2, XOR of all previous bytes up to the ninth (Whose packet) + packet body size (excluding header and CRC. Total number of bytes minus three), >= 0x0erc2, XOR of all previous bytes up to the ninth (Whose packet)may occupy several bytes

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };

// I create a class that inherits members of the Component and Cover classes.
class NiceBusT4 : public Component, public Cover {
  public:
	
    // drive settings
    bool autocls_flag; // Auto-closing - L1
    bool photocls_flag; // Close after Photo - L2
    bool alwayscls_flag; // Always close - L3
    bool init_ok = false; //  drive detection at power-on
    bool is_walky = false; // The position request command is different for walky
    bool is_robus = false; // Robus does not require periodic position requests.
    bool is_ro = false; // For ro600, the package with position status and movement status differs
		
    void setup() override;
    void loop() override;
    void dump_config() override; // to output information about the equipment to the log

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // long command
    void set_mcu(std::string command, std::string data_command); // command to the motor controller
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // drive status retrieval interval
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // Request for conditional current drive position
    void update_position(uint16_t newpos);  // Updating the current drive position

    uint32_t last_position_time{0};  // Time of last update of current position
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    CoverOperation last_published_op;  // Latest published status and position
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // Flag and position of the drive set position setting
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;	
    bool init_oxi_flag = false;	

	
    // variables for uart
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;  // maximum position of the encoder or timer
    uint16_t _pos_opn = 2048;  // Encoder or timer opening position, not for all drives.
    uint16_t _pos_cls = 0;  // encoder or timer closing position, not for all drives
    uint16_t _pos_usl = 0;  // conditional current position of the encoder or timer, not for all drives
    // settings for the header of the generated packet
    uint8_t addr_from[2] = {0x00, 0x66}; //Who is the packet from, Bust4 gateway address
    uint8_t addr_to[2]; // = 0x00ff;	 // to whom the packet is sent, the address of the drive controller that is controlled
    uint8_t addr_oxi[2]; // = 0x000a;	 // receiver address

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // preparing user-entered data for submission
	
    // generation of INF commands
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // all fields
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // for commands without data
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // for commands with data
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // for commands with address and without data
    	    
    // generating cmd commands
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);	    	
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // Let's analyze the status package
    
    void handle_char_(uint8_t c);                                         // received byte handler
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // received data processor
    bool validate_message_();                                         // function for checking received message

    std::vector<uint8_t> rx_message_;                          // here the received message is accumulated byte by byte
    std::queue<std::vector<uint8_t>> tx_buffer_;             // queue of commands to send
    bool ready_to_tx_{true};	                           // flag for the ability to send commands
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // unknown manufacturer during initialization
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;	
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;	

}; //Class

} // namespace bus_t4
} // namespace esphome
