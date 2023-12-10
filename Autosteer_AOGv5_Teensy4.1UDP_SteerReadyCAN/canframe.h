#ifndef __CANFRAME_H__
#define __CANFRAME_H__

#define TEENSY

#ifdef TEENSY
//#  include <FlexCAN_T4.h>
#else
#  include <can_common.h>
#endif

#ifdef TEENSY
//Union for parsing CAN bus data messages. Warning:
//Invokes type punning, but this works here because
//CAN bus data is always little-endian (or should be)
//and the ARM processor on these boards is also little
//endian.
typedef union {
    uint64_t uint64;
    uint32_t uint32[2]; 
    uint16_t uint16[4];
    uint8_t  uint8[8];
    int64_t int64;
    int32_t int32[2]; 
    int16_t int16[4];
    int8_t  int8[8];

    //deprecated names used by older code
    uint64_t value;
    struct {
        uint32_t low;
        uint32_t high;
    };
    struct {
        uint16_t s0;
        uint16_t s1;
        uint16_t s2;
        uint16_t s3;
    };
    uint8_t bytes[8];
    uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;
#endif

/* This is a thin wrapper class to contain either a due_can
 * CAN_FRAME or FlexCAN_T4 CAN_message_t object for manipulation
 */

class CANFrame
#ifdef TEENSY
	: public CAN_message_t
#else
	: public CAN_FRAME
#endif

{
private:

public:
#ifdef TEENSY
	CANFrame () {};
	CANFrame (const CAN_message_t &orig_frame) {
		//stinky, but accurate
		memcpy(this,&orig_frame,sizeof(CAN_message_t));
	}

	uint32_t get_id(void) { return id; };
	uint8_t get_extended(void) { return flags.extended; };
	BytesUnion *get_data(void) { return (BytesUnion *)buf; };
	uint8_t get_length(void) { return len; };

	//void set_data(uint8_t *data, int length) { memcpy(buf,data,length); };
	void set_id(uint32_t message_id) { id = message_id; };
	void set_extended(bool flag) {flags.extended = flag; };
	void set_length(uint8_t length) {len = length;};
#else
	CANFrame () {};
	CANFrame (const CAN_FRAME &orig_frame) {
		//stinky, but accurate
		memcpy(this,&orig_frame,sizeof(CAN_FRAME));
	}

	uint32_t get_id(void) { return id; };
	uint8_t get_extended(void) { return extended; };
	BytesUnion *get_data(void) { return &data; };
	uint8_t get_length(void) { return length; };

	void set_id(uint32_t message_id) { id = message_id };
	void set_extended(bool flag) { extended = flag; };
	void set_length(uint8_t len) { length = len; };
#endif

};


#endif

