#include "lidar_data.h"
#include "cmdIndex.h"
#include "livox_protocol.h"
#include "lwip/udp.h"

/*-- Global Variable Declarations --*/
lidar_data Data;
//uint8_t *pcd_ptr = 0x8000000;
#define SD_STORING	1
extern uint8_t seq_num;
extern struct udp_pcb *lidar_data_pcb;
extern struct udp_pcb *lidar_set_command_init;
extern uint16_t lidar_remote_port;
extern uint16_t gen_local_port;
extern ip_addr_t lidar_remote_ip;
extern ip_addr_t board_local_ip;
extern lidar_parameters read_lidar_ext;
static uint32_t count = 0;
extern struct udp_pcb *lidar_response_pcb;

static void udp_recv_lidar_data_traffic(void * arg,struct udp_pcb *lidar_pcb,
		                    struct pbuf *p,const ip_addr_t *addr, u16_t port)
{

	uint8_t ret_sd;
	if (!p || p->len == 0) {
		xil_printf("Invalid or empty packet received\n\r");
		return;
	}
	xil_printf("Printing the Length of the Point cloud data: %d\r\n",p->len);
	uint8_t lidar_data_buff[p->len];
	memcpy(lidar_data_buff,(uint8_t *)p->payload,p->len);
	uint32_t len = p->len;
//	pbuf_free(p);
	xil_printf("We've Received the Point Cloud Data from the sensor: %d \r\n",++count);

//	for(int i=0;i<len;i++)
//	{
//		xil_printf("%02X ",lidar_data_buff[i]);
//	}
//	xil_printf("\r\n");
#if 1
	/*In this stage we're storing the data in the sd card.*/
//	ret_sd = FfsSdPolledExample(lidar_data_buff);
	pbuf_free(p);
	count++;
//	if(ret_sd != XST_SUCCESS){xil_printf("Error: In writing to SD Card\r\n");}
	if(count>2500){tp_HandleStartStopSampling(lidar_pcb, STOP_SAMPLING);}
#endif
#if 1
	/*1. ---- Processing Stage ---- */
	if(lidar_data_buff[VERSION_IDX] == 0X05)
	{
		lidar_data *ddr4_mem = (lidar_data *)(DDR4_BASE_ADDR + count *
														sizeof(lidar_data));
		/*2. ---- Seperation Stage ----*/
		xil_printf("\r\nSeperation is Going on\r\n");

		/*3.---- Filling the Initial Data ----*/
		ddr4_mem->lidar_id = lidar_data_buff[LIDAR_ID_IDX];
		memcpy(ddr4_mem->status_code,&lidar_data_buff[STATUS_CODE_IDX],4);
		ddr4_mem->timestamp_type = lidar_data_buff[TIMESTAMP_TYPE_IDX];
		ddr4_mem->data_type = lidar_data_buff[DATA_TYPE_IDX];
		memcpy(ddr4_mem->timestamp,&lidar_data_buff[TIMESTAMP_IDX],8);
		xil_printf("Initial Data gets Filled\r\n");

		int pcd_idx = PCD_IDX;
		for(int i=0;i<96;i++)
		{
			memcpy(ddr4_mem->pcd[i].x_axis,&lidar_data_buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
//			xil_printf("Check 1\r\n");
			memcpy(ddr4_mem->pcd[i].y_axis,&lidar_data_buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
//			xil_printf("Check 2\r\n");
			memcpy(ddr4_mem->pcd[i].z_axis,&lidar_data_buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
//			xil_printf("Check 3\r\n");
			ddr4_mem->pcd[i].reflectivity = lidar_data_buff[pcd_idx++];
//			xil_printf("Check 4\r\n");
			ddr4_mem->pcd[i].tag = lidar_data_buff[pcd_idx++];
//			xil_printf("Check 5\r\n");
		}

		if(count > 2500)
		{
			/*After Storing the 3 Sets, we are stopping the sampling*/
			livox_gen_cmd_send(SAMPLING_CMD_ID, lidar_pcb, STOP_SAMPLING);
		}
		count++;
		xil_printf("%d set is Completed\r\n",count);
	}
#endif
}

void lidar_data_recv_init()
{
	err_t err;
	/* This PCB is for Lidar Data */
	lidar_data_pcb = udp_new();
	if(!lidar_data_pcb)
	{
		xil_printf("Error: In the Creation of PCB for Lidar Data\r\n");
//		return XST_FAILURE;
	}
	err = udp_bind(lidar_data_pcb,&board_local_ip,PTCLOUD_DATA_PORT); //
	if(err != ERR_OK)
	{
		xil_printf("Error: In Binding the Port of Lidar Data\r\n");
//		return XST_FAILURE;
	}
	/* Receive Callback for the Lidar Data */
	udp_recv(lidar_data_pcb, udp_recv_lidar_data_traffic, NULL);


}

void configure_lidar_packet(frame_header *fr_hdr,struct pbuf *packetbuf,uint8_t *buff,uint8_t req_len,
		                        uint8_t CRC32_idx,uint8_t lof,uint8_t seq_num,uint8_t cmd_set,uint8_t cmd_id)
{
	/*Initializing the Frame Header*/
	init_frame_header(fr_hdr, lof, seq_num, cmd_set, cmd_id);
	/*Setting the Initial Buffer*/
	prepare_frame_data(buff, fr_hdr);

	/* Crc16 Calculation */
	uint16_t initial_crc_16 	= CRC16_SEED;
	uint8_t crc16_buff[CRC16_LEN];
	memcpy(crc16_buff,buff,CRC16_LEN);
	fr_hdr->crc16 = crc_ccitt(initial_crc_16, crc16_buff,CRC16_LEN);
	/* Crc16 Calculation ended*/

	buff[CRC16_IDX]				=fr_hdr->crc16 & 0xFF;
	buff[CRC16_IDX+1]			=fr_hdr->crc16>>8 & 0xFF;

	/* Crc32 Calculation */
	uint8_t crc32_buff[CRC32_idx],crc_val = CRC32_idx;
	for(int i=0;i<crc_val;i++)
		crc32_buff[i] = buff[i];
	fr_hdr->crc32 = crc32(crc32_buff,CRC32_idx);

	buff[CRC32_idx]		 		=fr_hdr->crc32 & 0xFF;
	buff[CRC32_idx+1]		 	=(fr_hdr->crc32>>8 & 0xFF);
	buff[CRC32_idx+2]		 	=(fr_hdr->crc32>>16 & 0xFF);
	buff[CRC32_idx+3]		 	=(fr_hdr->crc32>>24 & 0xFF);

	/*----- PCB Configurations -----*/
	lidar_data_pcb->remote_ip 	=lidar_remote_ip;
	lidar_data_pcb->local_ip 	=board_local_ip;
	lidar_data_pcb->remote_port =lidar_remote_port;
	lidar_data_pcb->local_port 	=GEN_CMD_PORT;
	/*----- PBuf Configurations -----*/
	packetbuf->payload 			= buff;
	packetbuf->len 				= req_len;
	packetbuf->tot_len 			=req_len;

}
uint8_t set_mode_pkt_send()
{
	/*Local Variables*/
	frame_header fr_hdr;
	struct pbuf *set_mode_pbuf = pbuf_alloc(PBUF_TRANSPORT,SET_MODE_REQ_LEN,PBUF_RAM);
	uint8_t set_mode_buff[SET_MODE_REQ_LEN];

	configure_lidar_packet(&fr_hdr, set_mode_pbuf, set_mode_buff, SET_MODE_REQ_LEN, SET_MODE_CRC32_IDX,
												SET_MODE_LOF,seq_num++,LIDAR_CMD_SET,SET_MODE_CMD_ID);

	set_mode_buff[LIDAR_MODE_IDX]	 =NORMAL_MODE;//as of now.

	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, set_mode_pbuf, &lidar_remote_ip,lidar_remote_port);
	xil_printf("Set Mode Sent Successfully\r\n");
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(set_mode_pbuf);
	return 0;
}
uint8_t read_lidar_send()
{
	frame_header fr_hdr;
	uint8_t read_ext[READ_EXT_REQ_LEN];
	struct pbuf *read_lidar_pbuf = pbuf_alloc(PBUF_TRANSPORT,READ_EXT_REQ_LEN,PBUF_RAM);

	configure_lidar_packet(&fr_hdr, read_lidar_pbuf, read_ext, READ_EXT_REQ_LEN, READ_EXT_CRC32_IDX,
							READ_EXT_LOF, seq_num++,LIDAR_CMD_SET,READ_EXT_CMD_ID);

	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, read_lidar_pbuf, &lidar_remote_ip, lidar_remote_port);
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(read_lidar_pbuf);
	return 0;
}

uint8_t turn_on_off_suppression()
{
	frame_header fr_hdr;
	uint8_t turning_data[TURNING_REQ_LEN];
	struct pbuf *turning_pbuf = pbuf_alloc(PBUF_TRANSPORT,TURNING_REQ_LEN,PBUF_RAM);

	configure_lidar_packet(&fr_hdr, turning_pbuf, turning_data, TURNING_REQ_LEN, TURNING_CRC32_IDX,
							TURNING_LOF, seq_num++, LIDAR_CMD_SET, TURNING_CMD_ID);
	turning_data[TURNING_STATE_IDX]  =TURN_ON_STATE;	//As of now it's in On State.
	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, turning_pbuf, &lidar_remote_ip, lidar_remote_port);
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(turning_pbuf);
	return 0;
}

uint8_t write_lidar_send()
{
	frame_header fr_hdr;
	lidar_parameters write_lid;
	uint8_t write_ext[WRITE_EXT_REQ_LEN];
	struct pbuf *write_ext_pbuf = pbuf_alloc(PBUF_TRANSPORT,WRITE_EXT_REQ_LEN,PBUF_RAM);
	write_lid = read_lidar_ext;

	memcpy(&write_ext[ROLL_IDX], &write_lid.roll, 4);
	memcpy(&write_ext[PITCH_IDX], &write_lid.pitch, 4);
	memcpy(&write_ext[YAW_IDX], &write_lid.yaw, 4);
	memcpy(&write_ext[X_IDX], &write_lid.x, 4);
	memcpy(&write_ext[Y_IDX], &write_lid.y, 4);
	memcpy(&write_ext[Z_IDX], &write_lid.z, 4);

	configure_lidar_packet(&fr_hdr, write_ext_pbuf, write_ext, WRITE_EXT_REQ_LEN, WRITE_EXT_CRC32_IDX,
							WRITE_EXT_LOF,seq_num++,LIDAR_CMD_SET, WRITE_EXT_CMD_ID);

	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, write_ext_pbuf, &lidar_remote_ip, lidar_remote_port);
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(write_ext_pbuf);
	return 0;
}
uint8_t set_imu_push_freq(void)
{
	frame_header fr_hdr;
	uint8_t set_imu[SET_IMU_REQ_LEN];
	struct pbuf *set_imu_pbuf = pbuf_alloc(PBUF_TRANSPORT,SET_IMU_REQ_LEN,PBUF_RAM);
	configure_lidar_packet(&fr_hdr, set_imu_pbuf, set_imu, SET_IMU_REQ_LEN, SET_IMU_CRC32_IDX,
										SET_IMU_LOF, seq_num++, LIDAR_CMD_SET, SET_IMU_CMD_ID);
	set_imu[FREQUENCY_IDX] = FREQUENCY_200;
	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, set_imu_pbuf, &lidar_remote_ip, lidar_remote_port);
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(set_imu_pbuf);
	return 0;
}

uint8_t get_imu_push_freq(void)
{
	frame_header fr_hdr;
	uint8_t get_imu[GET_IMU_REQ_LEN];
	struct pbuf *get_imu_pbuf = pbuf_alloc(PBUF_TRANSPORT,GET_IMU_REQ_LEN,PBUF_RAM);
	configure_lidar_packet(&fr_hdr, get_imu_pbuf, get_imu, GET_IMU_REQ_LEN, GET_IMU_CRC32_IDX,
										GET_IMU_LOF, seq_num++, LIDAR_CMD_SET, GET_IMU_CMD_ID);
	/*----- Sending the Packet -----*/
	udp_sendto(lidar_response_pcb, get_imu_pbuf, &lidar_remote_ip, lidar_remote_port);
	/*----- Finally free the Packet Buffer -----*/
	pbuf_free(get_imu_pbuf);
	return 0;
}

/*---- Seperating the Received Point Cloud data ----*/
void lidar_data_processing(uint8_t *buff,uint16_t len)//recv_lidar_data<--
{
	if(buff[VERSION_IDX] == 0X05)
	{
		static uint8_t count = 0;
		xil_printf("\r\nSuccess: It's a Lidar Point Cloud Data\r\n");
		int pcd_idx = PCD_IDX;
		for(int i=0;i<96;i++)
		{
			memcpy(&Data.pcd[i].x_axis,&buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
			memcpy(&Data.pcd[i].y_axis,&buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
			memcpy(&Data.pcd[i].z_axis,&buff[pcd_idx],PCD_DATA_SIZE);
			pcd_idx += PCD_DATA_SIZE;
			Data.pcd[i].reflectivity = buff[pcd_idx++];
			Data.pcd[i].tag = buff[pcd_idx++];
		}
	}
}

void fill_initial_data(lidar_data *Data,uint8_t * buff)
{
	Data->lidar_id = buff[LIDAR_ID_IDX];
	memcpy(Data->status_code,&buff[STATUS_CODE_IDX],4);
	Data->timestamp_type = buff[TIMESTAMP_TYPE_IDX];
	Data->data_type = buff[DATA_TYPE_IDX];
	memcpy(&Data->data_type,&buff[DATA_TYPE_IDX],1);
}

void livox_lidar_send_cmd(uint8_t cmd_id)
{
	switch(cmd_id)
	{
//	case SET_MODE:
//		err_t err = set_mode_pkt_send();
//		if(!err)
//			xil_printf("Success: Set mode Packet Sent Successfully\r\n");
//		break;
	case READ_LIDAR_EXT:
		err_t err = read_lidar_send();
		if(!err)
			xil_printf("Success: Write Extrinsic Parameter Sent Successfully\r\n");
		break;
	case WRITE_LID_EXT:
		err = write_lidar_send();
		if(!err)
			xil_printf("Success: Write Extrinsic Parameter Sent Successfully\r\n");
		break;
	case TURN_ON_OFF_RF:
		err = turn_on_off_suppression();
		if(!err)
			xil_printf("Success: Turning State Packet Sent Successfully\r\n");
		break;

	}
}
