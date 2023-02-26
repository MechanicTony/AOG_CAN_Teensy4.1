
//All this is from torriem - AgOpen fourm

#include "canframe.h"

void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte* dest_addr)
{
	/* decode j1939 fields from 29-bit CAN id */
	*src_addr = 255;
	*dest_addr = 255;

	*priority = (int)((ID & 0x1C000000) >> 26);	//Bits 27,28,29

	*PGN = ID & 0x01FFFF00;		//Tony Note: Changed this from 0x00FFFF00 to decode PGN 129029, it now gets the 17th bit of PGN (Bits 9-25, Bit 26 is not used)
	*PGN = *PGN >> 8;

	ID = ID & 0x000000FF;	//Bits 1-8
	*src_addr = (int)ID;

	/* decode dest_addr if a peer to peer message */
	if ((*PGN > 0 && *PGN <= 0xEFFF) || (*PGN > 0x10000 && *PGN <= 0x1EFFF)) 
	{
		*dest_addr = (int)(*PGN & 0xFF);
		*PGN = *PGN & 0x01FF00;
	}
}

long j1939_encode(unsigned long pgn, byte priority, byte src_addr, byte dest_addr)
{

	long id;
	id = (long)(priority & 0x07) << 26; //three bits only	- Tony Note: Added long cast on priority others Arduino drops bits when shifting bits 26 left 
	/* if a peer to peer message, encode dest_addr */
	if ((pgn > 0 && pgn <= 0xEFFF) || (pgn > 0x10000 && pgn <= 0x1EFFF))
	{
		pgn = pgn & 0x01FF00;
		pgn = pgn | dest_addr;
	}
	id = id | (pgn << 8);
	id = id | src_addr;

	return id;
}

void sendISOBUS_65267_65256() {
	CANFrame msg;			//for generating gps messages
	msg.set_extended(true);
	msg.set_length(8);		//all our messages are going to be 8 bytes

	//GPS position, pgn 65267, Pivot Lat/Lon
	msg.set_id(j1939_encode(65267, 6, CANBUS_ModuleID, 255));
	msg.get_data()->uint32[0] = (pivotLat + 210.0) * 10000000;
	msg.get_data()->uint32[1] = (pivotLon + 210.0) * 10000000;
	ISO_Bus.write(msg);

	//Vehicle direction & speed, pgn 65256
	msg.set_id(j1939_encode(65256, 6, CANBUS_ModuleID, 255));
	msg.get_data()->uint16[0] = fixHeading * 128;			//Heading degress
	msg.get_data()->uint16[1] = gpsSpeed * 256;				//Speed km/hr
	msg.get_data()->uint16[2] = 200 * 128;					//Pitch zero
	msg.get_data()->uint16[3] = (pivotAltitude + 2500) * 8;	//Altitude meters - Only 12cm resolution ( pivotAltitude is 1cm resolution)
	ISO_Bus.write(msg);
}

void sendISOBUS_65254_129029() {
	CANFrame msg1;			//for generating gps messages
	msg1.set_extended(true);
	msg1.set_length(8);		//all our messages are going to be 8 bytes

	//GPS time, pgn 65254
	msg1.set_id(j1939_encode(65254, 6, CANBUS_ModuleID, 255));
	msg1.get_data()->uint64 = 0xFFFFFFFFFFFFFFFF;
	ISO_Bus.write(msg1);

	//GPS Infomation (GGA kind of in NEMA 2000), pgn 129029
	msg1.set_id(j1939_encode(129029, 6, CANBUS_ModuleID, 255));
	msg1.get_data()->uint64 = 0xFFFFFFFFFFFFFFFF;
	ISO_Bus.write(msg1);
}
