#include "livox_protocol.h"
#include "cmdIndex.h"
#include "commHandler.h"

extern uint8_t seq_num;

/*1. ---- Setting the Handshake Buffer ----*/
void set_hsbuffer(uint8_t *hs_buff)
{
	frame_header fr_hdr;
	handshake_req hs_req;

	memcpy(hs_req.user_ip,local_ip,4);
	init_frame_header(&fr_hdr, HANDSHAKE_LEN_OF_FRAME, seq_num++,
							  GENERAL_CMD_SET, HANDSHAKE_CMD_ID);

	hs_req.data_port			= PTCLOUD_DATA_PORT;
	hs_req.cmd_port				= GEN_CMD_PORT;
	hs_req.imu_port				= PTCLOUD_DATA_PORT;
	prepare_frame_data(hs_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,hs_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	hs_buff[CRC16_IDX]		 	= fr_hdr.crc16 & 0xFF;
	hs_buff[CRC16_IDX+1]	 	= (fr_hdr.crc16 >>8) & 0xFF;
	hs_buff[HS_IP_ADDR_IDX]  	= hs_req.user_ip[0];
	hs_buff[HS_IP_ADDR_IDX+1]	= hs_req.user_ip[1];
	hs_buff[HS_IP_ADDR_IDX+2]	= hs_req.user_ip[2];
	hs_buff[HS_IP_ADDR_IDX+3]	= hs_req.user_ip[3];
	hs_buff[DATA_PORT_IDX]	 	= hs_req.data_port &0xFF;
	hs_buff[DATA_PORT_IDX+1] 	= (hs_req.data_port>>8)&0xFF;
	hs_buff[CMD_PORT_IDX]    	= hs_req.cmd_port &0xFF;
	hs_buff[CMD_PORT_IDX+1]  	= (hs_req.cmd_port>>8)&0xFF;
	hs_buff[IMU_PORT_IDX]    	= hs_req.imu_port &0xFF;
	hs_buff[IMU_PORT_IDX+1]  	= (hs_req.imu_port>>8)&0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[HS_CRC32_IDX],crc_val = HS_CRC32_IDX;
	for(int i=0;i<crc_val;i++)
		crc32_buff[i] = hs_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,HS_CRC32_IDX);

	hs_buff[HS_CRC32_IDX]    	= fr_hdr.crc32 & 0xFF;
	hs_buff[HS_CRC32_IDX+1]	 	= (fr_hdr.crc32 >>8) & 0xFF;
	hs_buff[HS_CRC32_IDX+2]	 	= (fr_hdr.crc32 >>16)& 0xFF;
	hs_buff[HS_CRC32_IDX+3]	 	= (fr_hdr.crc32 >>24)& 0xFF;
}

/*2. ---- Setting the Sampling Buffer ----*/
void set_sampbuffer(uint8_t *samp_buffer, uint8_t Sampling)
{
	frame_header fr_hdr;
	init_frame_header(&fr_hdr, SAMPLING_LEN_OF_FRAME, seq_num++,
							GENERAL_CMD_SET, SAMPLING_CMD_ID);

	prepare_frame_data(samp_buffer, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,samp_buffer,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	samp_buffer[CRC16_IDX]		 	= fr_hdr.crc16 & 0xFF;
	samp_buffer[CRC16_IDX+1]	 	= (fr_hdr.crc16 >>8) & 0xFF;
	samp_buffer[SAMPLING_CTRL_IDX]  = Sampling;

	/* Crc32 Calculation */
	uint8_t crc32_buff[SAMPLING_CRC32_IDX],crc_val = SAMPLING_CRC32_IDX;
	for(int i=0;i<crc_val;i++)
		crc32_buff[i] = samp_buffer[i];
	fr_hdr.crc32 = crc32(crc32_buff,SAMPLING_CRC32_IDX);

	samp_buffer[SAMPLING_CRC32_IDX]    	=  fr_hdr.crc32 & 0xFF;
	samp_buffer[SAMPLING_CRC32_IDX+1]	= (fr_hdr.crc32 >>8) & 0xFF;
	samp_buffer[SAMPLING_CRC32_IDX+2]	= (fr_hdr.crc32 >>16)& 0xFF;
	samp_buffer[SAMPLING_CRC32_IDX+3]	= (fr_hdr.crc32 >>24)& 0xFF;

//	for(int i=0;i<SAMPLING_BUFFER_LEN;i++)
//		xil_printf("%02X ",samp_buffer[i]);
//	xil_printf("\r\n");
}

/*3. ---- Setting the Query Buffer ---- */
void set_querybuffer(uint8_t * query_buff)
{
	frame_header fr_hdr;
	/*Initializing the frame Header*/
	init_frame_header(&fr_hdr, QUERY_LEN_OF_FRAME, seq_num++, GENERAL_CMD_SET, QUERY_CMD_ID);

	/*Setting Initial Buffer*/
	prepare_frame_data(query_buff, &fr_hdr);

	query_buff[SOF_IDX]				=fr_hdr.sof;
	query_buff[PRO_VERS_IDX]		=fr_hdr.ver;
	query_buff[LEN_OF_FRAME_IDX]	=fr_hdr.len;
	query_buff[LEN_OF_FRAME_IDX+1]	=fr_hdr.len>>8;
	query_buff[CMD_TYPE_IDX]		=fr_hdr.cmd_type;
	query_buff[FRAME_SEQ_IDX]		=fr_hdr.seq_num;
	query_buff[FRAME_SEQ_IDX+1]		=fr_hdr.seq_num>>8;

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,query_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* ---- Crc16 Calculation ended ----*/

	query_buff[CRC16_IDX]			=fr_hdr.crc16 & 0xFF;
	query_buff[CRC16_IDX+1]			=fr_hdr.crc16>>8 & 0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[QUERY_CRC_32_IDX];
	for(int i=0;i<QUERY_CRC_32_IDX;i++)
		crc32_buff[i] = query_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,QUERY_CRC_32_IDX);
	/* ---- Crc32 Calculation ended ----*/

	query_buff[QUERY_CRC_32_IDX]			=fr_hdr.crc32 & 0xFF ;
	query_buff[QUERY_CRC_32_IDX+1]			=(fr_hdr.crc32>>8 & 0xFF);
	query_buff[QUERY_CRC_32_IDX+2]			=(fr_hdr.crc32>>16 & 0xFF);
	query_buff[QUERY_CRC_32_IDX+3]			=(fr_hdr.crc32>>24 & 0xFF);
}

/*4. ---- Setting the Coordinate Buffer ----*/
void set_changecoordbuff(uint8_t *coord_buff)
{
	frame_header fr_hdr;

	/*Initializing the frame Header*/
	init_frame_header(&fr_hdr, CH_COORD_LEN_OF_FRAME, seq_num++, GENERAL_CMD_SET, CH_COORD_CMD_ID);

	/*Setting Initial Buffer*/
	prepare_frame_data(coord_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,coord_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	coord_buff[CRC16_IDX]			=fr_hdr.crc16 & 0xFF;
	coord_buff[CRC16_IDX+1]			=fr_hdr.crc16>>8 & 0xFF;
	coord_buff[CH_COORD_TYPE_IDX]	=0 /*Cartesian or Spherical*/;

	/* Crc32 Calculation */
	uint8_t crc32_buff[CH_COORD_CRC32_IDX];
	for(int i=0;i<CH_COORD_CRC32_IDX;i++)
		crc32_buff[i] = coord_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,CH_COORD_CRC32_IDX);

	coord_buff[CH_COORD_CRC32_IDX]	=fr_hdr.crc32 & 0XFF;
	coord_buff[CH_COORD_CRC32_IDX+1]=fr_hdr.crc32>>8 & 0XFF;
	coord_buff[CH_COORD_CRC32_IDX+2]=fr_hdr.crc32>>16 & 0XFF;
	coord_buff[CH_COORD_CRC32_IDX+3]=fr_hdr.crc32>>24 & 0XFF;

}
/*5. ---- Setting the Disconnect Buffer ----*/
void set_disconnectbuff(uint8_t *dis_buff)
{
	frame_header fr_hdr;
	/*Initializing the Frame Header*/
	init_frame_header(&fr_hdr, DIS_REQ_LEN, seq_num++, GENERAL_CMD_SET, DIS_CMD_ID);

	/*Setting Initial Buffer*/
	prepare_frame_data(dis_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,dis_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	dis_buff[CRC16_IDX]			=fr_hdr.crc16 & 0xFF;
	dis_buff[CRC16_IDX+1]		=fr_hdr.crc16>>8 & 0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[DIS_CRC32_IDX];
	for(int i=0;i<HEART_CRC32_IDX;i++)
		crc32_buff[i] = dis_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,DIS_CRC32_IDX);

	dis_buff[DIS_CRC32_IDX]		=fr_hdr.crc32 & 0XFF;
	dis_buff[DIS_CRC32_IDX+1]	=fr_hdr.crc32>>8 & 0XFF;
	dis_buff[DIS_CRC32_IDX+2]	=fr_hdr.crc32>>16 & 0XFF;
	dis_buff[DIS_CRC32_IDX+3]	=fr_hdr.crc32>>24 & 0XFF;
}
/*6. ---- Setting the Hearbeat Buffer ----*/
void set_heartbuff(uint8_t * heart_buff)
{
	frame_header fr_hdr;
	/*Initializing the Frame Header*/
	init_frame_header(&fr_hdr, HEARTBEAT_REQ_LEN, seq_num++, GENERAL_CMD_SET, HEARTBEAT_CMD_ID);

	/*Setting Initial Buffer*/
	prepare_frame_data(heart_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,heart_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	heart_buff[CRC16_IDX]		=fr_hdr.crc16 & 0xFF;
	heart_buff[CRC16_IDX+1]		=fr_hdr.crc16>>8 & 0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[HEART_CRC32_IDX];
	for(int i=0;i<HEART_CRC32_IDX;i++)
		crc32_buff[i] = heart_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,HEART_CRC32_IDX);

	heart_buff[HEART_CRC32_IDX]	=fr_hdr.crc32 & 0XFF;
	heart_buff[HEART_CRC32_IDX+1]	=fr_hdr.crc32>>8 & 0XFF;
	heart_buff[HEART_CRC32_IDX+2]	=fr_hdr.crc32>>16 & 0XFF;
	heart_buff[HEART_CRC32_IDX+3]	=fr_hdr.crc32>>24 & 0XFF;
}
void set_getimu_buff(uint8_t *getImu_buff)
{
	frame_header fr_hdr;
	/*Initializing the Frame Header*/
	init_frame_header(&fr_hdr, GET_IMU_REQ_LEN, seq_num++, LIDAR_CMD_SET, GET_IMU_CMD_ID);
	/*Setting Initial Buffer*/
	prepare_frame_data(getImu_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,getImu_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	getImu_buff[CRC16_IDX]		=fr_hdr.crc16 & 0xFF;
	getImu_buff[CRC16_IDX+1]	=fr_hdr.crc16>>8 & 0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[GET_IMU_CRC32_IDX];
	for(int i=0;i<GET_IMU_CRC32_IDX;i++)
		crc32_buff[i] = getImu_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,GET_IMU_CRC32_IDX);

	getImu_buff[GET_IMU_CRC32_IDX]	=fr_hdr.crc32 & 0XFF;
	getImu_buff[GET_IMU_CRC32_IDX+1]=fr_hdr.crc32>>8 & 0XFF;
	getImu_buff[GET_IMU_CRC32_IDX+2]=fr_hdr.crc32>>16 & 0XFF;
	getImu_buff[GET_IMU_CRC32_IDX+3]=fr_hdr.crc32>>24 & 0XFF;
}

void set_setimu_buff(uint8_t *setImu_buff)
{
	frame_header fr_hdr;
	/*Initializing the Frame Header*/
	init_frame_header(&fr_hdr, SET_IMU_REQ_LEN, seq_num++, LIDAR_CMD_SET, SET_IMU_CMD_ID);
	/*Setting Initial Buffer*/
	prepare_frame_data(setImu_buff, &fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16  = CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,setImu_buff,CRC16_LEN);
	fr_hdr.crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	setImu_buff[CRC16_IDX]		=fr_hdr.crc16 & 0xFF;
	setImu_buff[CRC16_IDX+1]	=fr_hdr.crc16>>8 & 0xFF;
	setImu_buff[FREQUENCY_IDX]	=FREQUENCY_200;

	/* Crc32 Calculation */
	uint8_t crc32_buff[SET_IMU_CRC32_IDX];
	for(int i=0;i<SET_IMU_CRC32_IDX;i++)
		crc32_buff[i] = setImu_buff[i];
	fr_hdr.crc32 = crc32(crc32_buff,SET_IMU_CRC32_IDX);

	setImu_buff[SET_IMU_CRC32_IDX]	=fr_hdr.crc32 & 0XFF;
	setImu_buff[SET_IMU_CRC32_IDX+1]=fr_hdr.crc32>>8 & 0XFF;
	setImu_buff[SET_IMU_CRC32_IDX+2]=fr_hdr.crc32>>16 & 0XFF;
	setImu_buff[SET_IMU_CRC32_IDX+3]=fr_hdr.crc32>>24 & 0XFF;
}



