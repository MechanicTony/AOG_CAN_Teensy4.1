
//All this is from torriem - AgOpen fourm

#include "canframe.h"

void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte* dest_addr)
{
	/* decode j1939 fields from 29-bit CAN id */
	*src_addr = 255;
	*dest_addr = 255;

	*priority = (int)((ID & 0x1C000000) >> 26);	//Bits 27,28,29

	*PGN = ID & 0x01FFFF00;	//Tony Note: Changed this from 0x00FFFF00 to decode PGN 129029, it now gets the 17th bit of the 18 bit PGN (Bits 9-25, Bit 26 is not used)
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

void sendISOBUS_65267_65256() 
{
	CANFrame msg;
	msg.set_extended(true);
	msg.set_length(8);

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

void sendISOBUS_65254() 
{
	CANFrame msg1;
	msg1.set_extended(true);
	msg1.set_length(8);

	//GPS time, pgn 65254
	msg1.set_id(j1939_encode(65254, 6, CANBUS_ModuleID, 255));
	msg1.get_data()->uint64 = 0xFFFFFFFFFFFFFFFF;
	ISO_Bus.write(msg1);

}

void sendISOBUS_129029() 
{
	Update_N2K_129029_Buffer();

	static int frameCount = 0;
	uint8_t packetIdentifier;

	CAN_message_t n2k_fastPackets;
	n2k_fastPackets.flags.extended = true;
	n2k_fastPackets.len = 8;

	//GPS Data, NMEA 2000 pgn 129029
	n2k_fastPackets.id = (j1939_encode(129029, 6, CANBUS_ModuleID, 255));

	packetIdentifier = (frameCount << 5);

	//1 of 7
	n2k_fastPackets.buf[0] = packetIdentifier;
	n2k_fastPackets.buf[1] = 48;						//Data length = 48 bytes, 7 CAN messages (6 @ 7 bytes plus 1 @ 6 bytes)
	n2k_fastPackets.buf[2] = CANBUS_ModuleID;			//Start of PGN 129029 data, this is the SID
	n2k_fastPackets.buf[3] = N2K_129029_Data[1];		//Days x2
	n2k_fastPackets.buf[4] = N2K_129029_Data[2];
	n2k_fastPackets.buf[5] = N2K_129029_Data[3];		//Seconds x4
	n2k_fastPackets.buf[6] = N2K_129029_Data[4];
	n2k_fastPackets.buf[7] = N2K_129029_Data[5];

	ISO_Bus.write(n2k_fastPackets);

	//2 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[6];
	n2k_fastPackets.buf[2] = N2K_129029_Data[7];		//Latitude x8
	n2k_fastPackets.buf[3] = N2K_129029_Data[8];
	n2k_fastPackets.buf[4] = N2K_129029_Data[9];
	n2k_fastPackets.buf[5] = N2K_129029_Data[10];
	n2k_fastPackets.buf[6] = N2K_129029_Data[11];
	n2k_fastPackets.buf[7] = N2K_129029_Data[12];

	ISO_Bus.write(n2k_fastPackets);

	//3 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[13];
	n2k_fastPackets.buf[2] = N2K_129029_Data[14];
	n2k_fastPackets.buf[3] = N2K_129029_Data[15];		//Longitude x8
	n2k_fastPackets.buf[4] = N2K_129029_Data[16];
	n2k_fastPackets.buf[5] = N2K_129029_Data[17];
	n2k_fastPackets.buf[6] = N2K_129029_Data[18];
	n2k_fastPackets.buf[7] = N2K_129029_Data[19];

	ISO_Bus.write(n2k_fastPackets);

	//4 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[20];
	n2k_fastPackets.buf[2] = N2K_129029_Data[21];
	n2k_fastPackets.buf[3] = N2K_129029_Data[22];
	n2k_fastPackets.buf[4] = N2K_129029_Data[23];;		//Altitude x8
	n2k_fastPackets.buf[5] = N2K_129029_Data[24];
	n2k_fastPackets.buf[6] = N2K_129029_Data[25];
	n2k_fastPackets.buf[7] = N2K_129029_Data[26];

	ISO_Bus.write(n2k_fastPackets);

	//5 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[27];
	n2k_fastPackets.buf[2] = N2K_129029_Data[28];
	n2k_fastPackets.buf[3] = N2K_129029_Data[29];
	n2k_fastPackets.buf[4] = N2K_129029_Data[30];
	n2k_fastPackets.buf[5] = N2K_129029_Data[31];		//GNSS type & Fix type
	n2k_fastPackets.buf[6] = N2K_129029_Data[32];		//Integrity 2 bit, reserved 6 bits
	n2k_fastPackets.buf[7] = N2K_129029_Data[33];		//Satellites

	ISO_Bus.write(n2k_fastPackets);

	//6 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[34];		//HDOP x2
	n2k_fastPackets.buf[2] = N2K_129029_Data[35];
	n2k_fastPackets.buf[3] = N2K_129029_Data[36];		//PDOP x2
	n2k_fastPackets.buf[4] = N2K_129029_Data[37];
	n2k_fastPackets.buf[5] = N2K_129029_Data[38];;		//4 byte double, GeoidalSeparation
	n2k_fastPackets.buf[6] = N2K_129029_Data[39];
	n2k_fastPackets.buf[7] = N2K_129029_Data[40];

	ISO_Bus.write(n2k_fastPackets);

	//7 of 7
	n2k_fastPackets.buf[0] = n2k_fastPackets.buf[0] + 1;
	n2k_fastPackets.buf[1] = N2K_129029_Data[41];
	n2k_fastPackets.buf[2] = N2K_129029_Data[42];
	n2k_fastPackets.buf[3] = N2K_129029_Data[43];		//2 byte, Ref station & ID
	n2k_fastPackets.buf[4] = N2K_129029_Data[44];
	n2k_fastPackets.buf[5] = N2K_129029_Data[45];		//2 byte, double corr age
	n2k_fastPackets.buf[6] = N2K_129029_Data[46];
	n2k_fastPackets.buf[7] = 0xFF;						//Spare

	ISO_Bus.write(n2k_fastPackets);

	if(frameCount++ > 7) frameCount = 0;	//We can only use 3 bits as the frame counter
}

void Update_N2K_129029_Buffer()
{
	byte tempDouble[8];
	byte tempFloat[4];
	byte tempTwoByte[2];

	int16_t tempInt16;
	int32_t tempInt32;
	int64_t tempInt64;

	//2 byte uint Days since 1970 (19415 days = 0x4BD7)
	if (N2K_129029_Data[1] == 0 && N2K_129029_Data[2] == 0)		//If these are zero the ZDA NMEA message must be missing
	{
		N2K_129029_Data[1] = 0xD7;
		N2K_129029_Data[2] = 0x4B;
	}

	//4 byte double UTC seconds since midnight
	tempInt32 = utcTime * 10000;
	memcpy(&tempFloat, &tempInt32, 4);
	N2K_129029_Data[3] = tempFloat[0];
	N2K_129029_Data[4] = tempFloat[1];
	N2K_129029_Data[5] = tempFloat[2];
	N2K_129029_Data[6] = tempFloat[3];

	//8 byte double Latitude
	tempInt64 = pivotLat * 10000000000000000;
	memcpy(&tempDouble, &tempInt64,8);
	N2K_129029_Data[7] = tempDouble[0];
	N2K_129029_Data[8] = tempDouble[1];
	N2K_129029_Data[9] = tempDouble[2];
	N2K_129029_Data[10] = tempDouble[3];
	N2K_129029_Data[11] = tempDouble[4];
	N2K_129029_Data[12] = tempDouble[5];
	N2K_129029_Data[13] = tempDouble[6];
	N2K_129029_Data[14] = tempDouble[7];

	//8 byte double Longitude
	tempInt64 = pivotLon * 10000000000000000;
	memcpy(&tempDouble, &tempInt64, 8);
	N2K_129029_Data[15] = tempDouble[0];
	N2K_129029_Data[16] = tempDouble[1];
	N2K_129029_Data[17] = tempDouble[2];
	N2K_129029_Data[18] = tempDouble[3];
	N2K_129029_Data[19] = tempDouble[4];
	N2K_129029_Data[20] = tempDouble[5];
	N2K_129029_Data[21] = tempDouble[6];
	N2K_129029_Data[22] = tempDouble[7];

	//8 byte double Altitude
	tempInt64 = pivotAltitude * 10000000000000000;
	memcpy(&tempDouble, &tempInt64, 8);
	N2K_129029_Data[23] = tempDouble[0];
	N2K_129029_Data[24] = tempDouble[1];
	N2K_129029_Data[25] = tempDouble[2];
	N2K_129029_Data[26] = tempDouble[3];
	N2K_129029_Data[27] = tempDouble[4];
	N2K_129029_Data[28] = tempDouble[5];
	N2K_129029_Data[29] = tempDouble[6];
	N2K_129029_Data[30] = tempDouble[7];

	//Fix Type
	N2K_129029_Data[31] = fixTypeGGA;

	// Integrity 2 bit, reserved 6 bits
	N2K_129029_Data[32] = 1 | 0xfc;					// Integrity 2 bit, reserved 6 bits

	//Satellites
	N2K_129029_Data[33] = satsGGA;

	//HDOP 
	tempInt16 = hdopGGA * 100;
	memcpy(&tempTwoByte, &tempInt16, 2);
	N2K_129029_Data[34] = tempTwoByte[0];
	N2K_129029_Data[35] = tempTwoByte[1];

	//PDOP - Just copy HDOP
	N2K_129029_Data[36] = N2K_129029_Data[34];
	N2K_129029_Data[37] = N2K_129029_Data[35];

	//4 byte double, Geoidal Separation
	tempInt32 = geoidalGGA * 100;
	memcpy(&tempFloat, &tempInt32, 4);

	N2K_129029_Data[38] = tempFloat[0];
	N2K_129029_Data[39] = tempFloat[1];
	N2K_129029_Data[40] = tempFloat[2];
	N2K_129029_Data[41] = tempFloat[3];
	
	// RTK age
	tempInt16 = rtkAgeGGA * 100;
	memcpy(&tempTwoByte, &tempInt16, 2);

	if (tempInt16 == 0)
	{
		N2K_129029_Data[42] = 0xFF;
		N2K_129029_Data[43] = 0xFF;
		N2K_129029_Data[44] = 0xFF;
		N2K_129029_Data[45] = 0xFF;
		N2K_129029_Data[46] = 0xFF;
	}

	else
	{
		N2K_129029_Data[42] = 0x01;		//Ref station used

		// Ref station type & ID (Type GPSGLONASS = 2, ID = 0) >>> ((int)ReferenceStationType) & 0x0f) | ReferenceSationID<<4
		N2K_129029_Data[43] = 0x00;
		N2K_129029_Data[44] = 0x02;

		N2K_129029_Data[45] = tempTwoByte[0];
		N2K_129029_Data[46] = tempTwoByte[1];
	}

	N2K_129029_Data[47] = 0xFF;
}