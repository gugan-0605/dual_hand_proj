#include "commHandler.h"


/*     Note
 *1. Handling BC msg
 *2. Handling and sending Sampling
 *3. Handling and Sending Query Device
 *4. Handling and Sending handshake Packet*/

/* Global Variables */
extern uint16_t lidar_remote_port;
extern uint16_t gen_local_port;
extern ip_addr_t lidar_remote_ip;
extern ip_addr_t board_local_ip;
extern struct udp_pcb *lidar_response_pcb;


/*1. ---- Handling the Broadcast message ---- */
void tp_HandleLivoxMsg(uint8_t *msg, uint16_t len, struct udp_pcb *pcb)
{
	xil_printf("Processing broadcast message...\r\n");


	/*---- Local Variables ----*/
	uint8_t crc16_check_buff[CRC16_LEN];
	uint8_t crc32_check_buff[CRC32_LEN_BC];
	uint16_t temp_crc16;
	uint32_t temp_crc32;
	uint8_t final_crc16_buf[2];
	uint8_t final_crc32_buf[4];
	uint8_t bc_unique_id[UNIQUE_ID_SIZE];
	/* --------------------------------------*/

	/*1.---- Initial CRC16 Check ---- */
	memcpy(crc16_check_buff,msg,CRC16_LEN);
	temp_crc16 = crc16_check(crc16_check_buff);
	final_crc16_buf[0] = temp_crc16 & 0xFF;
	final_crc16_buf[1] = (temp_crc16>>8) & 0xFF;
	if (memcmp(final_crc16_buf,&msg[CRC16_IDX], 2) == 0)
	{
		xil_printf("Success: CRC16 Matches\r\n");
	}

	/*2.---- Copying Unique Id ----*/
	memcpy(bc_unique_id,&msg[UNIQUE_ID_IDX],UNIQUE_ID_SIZE);
	xil_printf("Only the Unique Id: ");
	for(int i=0;i<UNIQUE_ID_SIZE;i++)
		xil_printf("%02X ",bc_unique_id[i]);

	/*3.---- Initial CRC32 Check ----*/
	memcpy(crc32_check_buff,msg,CRC32_LEN_BC);
	temp_crc32 = crc32_check(crc32_check_buff);
	for (int i=0; i< 4; i++)
		final_crc32_buf[i] = (temp_crc32>>(i*8))&0xFF;

	if (memcmp(final_crc32_buf, &msg[CRC32_IDX_BC], 4) == 0)
	{
		xil_printf("Success: CRC32 Matches\r\n");
	}
	/*4. ---- After receiving the Bc's UDP, we have to remove that ----*/
	udp_remove(pcb);
	xil_printf("Removed the BC PCB\r\n");

	/*5.---- Set Receive Callback for Command and Lidar Data ----*/
	lidar_response_handler_init();// we can send any commands and to receive the response
//	lidar_set_command_init();
	lidar_data_recv_init();
	/*6.---- Calling Send_gen_cmd Function by Passing ----*/
	livox_gen_cmd_send(HANDSHAKE_CMD_ID, lidar_response_pcb,(void *)NULL);
}

/*2. ---- Handling and Sending the Handshake Packet ----*/

void tp_HandlingHandshakeMessage(struct udp_pcb *pcb)
{
	uint8_t hs_request_buff[HANDSHAKE_PKT_LEN];
	memset(hs_request_buff, 0, sizeof(hs_request_buff));
	set_hsbuffer(hs_request_buff);

	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the hs_request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(hs_request_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, hs_request_buff, sizeof(hs_request_buff));
		resp_pbuf->len     = HANDSHAKE_PKT_LEN ;
		resp_pbuf->tot_len = HANDSHAKE_PKT_LEN;

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK) {
			xil_printf("hs_request_buff sent successfully.\r\n");
		} else {
			xil_printf("Failed to send hs_request_buff.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

/*3. ---- Handling and sending the Sampling Buffer ---- */

void tp_HandleStartStopSampling(struct udp_pcb *pcb,uint8_t Sampling)
{
	uint8_t samp_buffer[SAMPLING_REQ_LEN];
	set_sampbuffer(samp_buffer,Sampling);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the sampling request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(samp_buffer), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, samp_buffer, sizeof(samp_buffer));
		resp_pbuf->len     = SAMPLING_REQ_LEN ;
		resp_pbuf->tot_len = SAMPLING_REQ_LEN;

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK) {
			xil_printf("samp_buffer sent successfully.\r\n");
		} else {
			xil_printf("Failed to send samp_buffer.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

/*4. ---- Handling and sending the Query Device Packet ----*/
void tp_HandleQueryDevice(struct udp_pcb *pcb)
{
	uint8_t query_buffer[QUERY_REQ_LEN];
	set_querybuffer(query_buffer);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the query request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(query_buffer), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, query_buffer, sizeof(query_buffer));
		resp_pbuf->len     = QUERY_REQ_LEN ;
		resp_pbuf->tot_len = QUERY_REQ_LEN;

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK) {
			xil_printf("query_buffer sent successfully.\r\n");
		} else {
			xil_printf("Failed to send query_buffer.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}
void tp_HandleChangeCoordinate(struct udp_pcb *pcb)
{

	uint8_t coord_buff[CH_COORD_REQ_LEN];
	set_changecoordbuff(coord_buff);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the ch-coordinate_request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(coord_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, coord_buff, sizeof(coord_buff));
		resp_pbuf->len     = CH_COORD_REQ_LEN ;
		resp_pbuf->tot_len = CH_COORD_REQ_LEN;

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK) {
			xil_printf("coord_buff sent successfully.\r\n");
		} else {
			xil_printf("Failed to send coord_buff.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

void tp_HandleDisconnect(struct udp_pcb *pcb)
{
	uint8_t dis_buff[DIS_REQ_LEN];
	set_disconnectbuff(dis_buff);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the disconnect request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(dis_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, dis_buff, sizeof(dis_buff));
		resp_pbuf->len     = DIS_REQ_LEN ;
		resp_pbuf->tot_len = DIS_REQ_LEN;

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK) {
			xil_printf("dis_buff sent successfully.\r\n");
		} else {
			xil_printf("Failed to send dis_buff.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

void tp_HandleHeartBeat(struct udp_pcb *pcb)
{
	uint8_t heart_buff[HEARTBEAT_REQ_LEN];
	set_heartbuff(heart_buff);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = gen_local_port;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the disconnect request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(heart_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, heart_buff, sizeof(heart_buff));
		resp_pbuf->len     = HEARTBEAT_REQ_LEN ;	//change maad bekku
		resp_pbuf->tot_len = HEARTBEAT_REQ_LEN; 	//cmb

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK)
		{
			xil_printf("heart beat sent successfully.\r\n");
		}
		else
		{
			xil_printf("Failed to send heart beat.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

void tp_GetImuPushFreq(struct udp_pcb *pcb)
{
	uint8_t getimu_buff[GET_IMU_REQ_LEN];
	set_getimu_buff(getimu_buff);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = GEN_CMD_PORT;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the disconnect request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(getimu_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, getimu_buff, sizeof(getimu_buff));
		resp_pbuf->len     = GET_IMU_REQ_LEN ;	//change maad bekku
		resp_pbuf->tot_len = GET_IMU_REQ_LEN; 	//cmb

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK)
		{
			xil_printf("GetIMU sent successfully.\r\n");
		}
		else
		{
			xil_printf("Failed to send GetImu.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}

void tp_SetImuPushFreq(struct udp_pcb *pcb)
{
	uint8_t setimu_buff[SET_IMU_REQ_LEN];
	set_setimu_buff(setimu_buff);
	/*5. ---- PCB Configurations ----*/
	pcb->remote_port  = lidar_remote_port;
	pcb->local_port   = GEN_CMD_PORT;
	pcb->remote_ip    = lidar_remote_ip;
	pcb->local_ip     = board_local_ip;
	/*6. ---- Send the disconnect request ----*/
	struct pbuf *resp_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(setimu_buff), PBUF_RAM);

	if (resp_pbuf != NULL)
	{
		/*7. ----Packet Buffer Configurations ----*/
		memcpy(resp_pbuf->payload, setimu_buff, sizeof(setimu_buff));
		resp_pbuf->len     = SET_IMU_REQ_LEN ;	//change maad bekku
		resp_pbuf->tot_len = SET_IMU_REQ_LEN; 	//cmb

		/*8. ---- Sending the Packet ----*/
		if (udp_sendto(pcb, resp_pbuf, &lidar_remote_ip, lidar_remote_port) == ERR_OK)
		{
			xil_printf("SetImu sent successfully.\r\n");
		}
		else
		{
			xil_printf("Failed to send SetImu.\r\n");
		}
		/*9. ---- Free the buffer ----*/
		pbuf_free(resp_pbuf);
	}
	else
	{
		xil_printf("Failed to allocate pbuf for response.\r\n");
	}
}
