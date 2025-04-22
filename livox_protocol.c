 #include "livox_protocol.h"

//#include "commHandler.h"
#include "lidar_data.h"
#include "cmdIndex.h"
#include "lwip/udp.h"

uint8_t seq_num = 0;
uint8_t test_code;
uint8_t cmd_data_buff[50];
uint8_t cmd_data = 0;

extern struct netif *netif;
struct udp_pcb *lidar_cmd_set_pcb;
extern struct eth_addr lidar_remote_mac;
extern uint8_t livox_comm_active;
struct udp_pcb *lidar_data_pcb;
extern struct udp_pcb *lidar_response_pcb;
extern uint16_t lidar_remote_port;
extern uint16_t gen_local_port;
extern ip_addr_t lidar_remote_ip;
extern ip_addr_t board_local_ip;
lidar_parameters read_lidar_ext;
uint32_t flag_fr_pcd = FALSE;

uint8_t local_ip[4] 	 = {0xc0, 0xa8, 0x01, 0x0a};
uint16_t livox_data_port = 0x52c3;//livox add
uint16_t livox_cmd_port  = 0x51c3;
uint16_t livox_imu_port  = 0x52c3;

void init_frame_header(frame_header *fr_hdr, uint16_t len, uint16_t seq_num, uint8_t cmd_set, uint8_t cmd_id)
{
    fr_hdr->sof = SOF;
    fr_hdr->ver = PRO_VERS;
    fr_hdr->len = len;
    fr_hdr->seq_num = seq_num;
    fr_hdr->cmd_type = GEN_SEND_CMD_TYPE;
    fr_hdr->cmd_set = cmd_set;
    fr_hdr->cmd_id = cmd_id;
}
void prepare_frame_data(uint8_t *frame_data, frame_header *fr_hdr)
{
    frame_data[SOF_IDX] = fr_hdr->sof;
    frame_data[PRO_VERS_IDX] = fr_hdr->ver;
    frame_data[LEN_OF_FRAME_IDX] = fr_hdr->len;
    frame_data[LEN_OF_FRAME_IDX + 1] = fr_hdr->len >> 8;
    frame_data[CMD_TYPE_IDX] = fr_hdr->cmd_type;
    frame_data[FRAME_SEQ_IDX] = fr_hdr->seq_num;
    frame_data[FRAME_SEQ_IDX + 1] = fr_hdr->seq_num >> 8;
    frame_data[CMD_SET_IDX] = fr_hdr->cmd_set;
    frame_data[CMD_ID_IDX] = fr_hdr->cmd_id;
}


/*--------------------------------------------------------
 * 				Livox General Command Sending
 *-------------------------------------------------------*/
void livox_gen_cmd_send(uint8_t command_id,struct udp_pcb *pcb,uint8_t Samp)
{
	xil_printf("Entered into the Switch case\r\n");
	switch(command_id)
	{
	case HANDSHAKE_CMD_ID:
		tp_HandlingHandshakeMessage(pcb);
		break;
	case SAMPLING_CMD_ID:
		tp_HandleStartStopSampling(pcb,Samp);
		break;
	case QUERY_CMD_ID:
		tp_HandleQueryDevice(pcb);
		break;
	case CH_COORD_CMD_ID:
		tp_HandleChangeCoordinate(pcb);
		break;
	case DIS_CMD_ID:
		tp_HandleDisconnect(pcb);
		break;
	}
}
/*--------------------------------------------------------------
 * 					Lidar Command Handling
 *--------------------------------------------------------------*/

static void udp_recv_lidar_response(void * arg,struct udp_pcb *lidar_cmd_pcb,
						struct pbuf *p,const ip_addr_t *addr, uint16_t port)

{
	s32_t recv_id;
	if (!p || p->len == 0) {
		xil_printf("Invalid or empty packet received\n\r");
		return;
	}
	if((p->payload)!=NULL)
	{
		cmd_data = 1;
		memcpy(cmd_data_buff,(uint8_t *)p->payload,p->len);
	}

	recv_id = ntohl(*((int *)(p->payload)));
	xil_printf(" Got Response from the Sensor\r\n");
	xil_printf("Recv id is: %u\r\n",recv_id);

	uint8_t lidar_response_buff[p->len];
	memcpy(lidar_response_buff,(uint8_t *)p->payload,p->len);

	xil_printf("Received Response is : ");
	for(int i=0;i<(p->len);i++)
		xil_printf("%02X ",lidar_response_buff[i]);
	xil_printf("\r\n");

	pbuf_free(p);
	lidar_response_processing(lidar_response_buff);
}

void lidar_response_handler_init(void)
{
	err_t err;
	lidar_response_pcb = udp_new();
	if(!lidar_response_pcb)
	{
		xil_printf("Error: In the Creation of PCB for Lidar Cmd\r\n");
		return;
	}
	err = udp_bind(lidar_response_pcb,&board_local_ip,GEN_CMD_PORT);
	if(err != ERR_OK)
	{
		xil_printf("Error: In Binding the Port of Lidar Data\r\n");
		return;
	}
	/* Receive Callback for the ++Lidar response for command  */
	udp_recv(lidar_response_pcb, udp_recv_lidar_response, NULL);//<--
}

//void lidar_set_command_init(void)
//{
//	err_t err;
//	lidar_cmd_set_pcb = udp_new();
//	if(!lidar_cmd_set_pcb)
//	{
//		xil_printf("Error: In the Creation of PCB for Lidar Command Set\r\n");
//		return;
//	}
//	err = udp_bind(lidar_cmd_set_pcb,&board_local_ip,LIDAR_CMD_PORT);
//	if(err != ERR_OK)
//	{
//		xil_printf("Error: In Binding the Port of Lidar Command Set\r\n");
//		return;
//	}
//	udp_recv(lidar_cmd_set_pcb,udp_recv_lidar_response,NULL);
//}

uint8_t lidar_response_processing(uint8_t *resp_buff)
{
//	xil_printf("Received Response Message\r\n");
	uint8_t resp_cmd_type = resp_buff[CMD_TYPE_IDX];
	if(resp_cmd_type!= 0x01)
	{
		xil_printf("Error: It's not a Acknowledgment\r\n");
		return -1;
	}
	uint8_t resp_cmd_set = resp_buff[CMD_SET_IDX];
	uint8_t resp_cmd_id  = resp_buff[CMD_ID_IDX];
	if(resp_cmd_set == 0x00)
	{
		xil_printf("Success: It's a General Command Set\r\n");

		switch(resp_cmd_id)
		{
		case HANDSHAKE_CMD_ID:
		{
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: Handshake Response Successful\r\n");
				/*Upon Receiving handshake response setting livox comm active as TRUE*/
				livox_comm_active = TRUE;
//				tp_GetImuPushFreq(lidar_response_pcb);
//				tp_SetImuPushFreq(lidar_response_pcb);
//				uint8_t status = set_mode_pkt_send();
//				if(status == 0){xil_printf("Set Mode Sent Kuskusful\r\n");}
				xil_printf("Livox Communication Active has set as TRUE\r\n");
#ifdef TEST_CODE
				test_function();
#endif
//				/*Send Heartbeat Command*/
//				tp_HandleHeartBeat(lidar_response_pcb);
				/* Send Start Sampling code */
//				set_mode_pkt_send(lidar);
//				tp_GetImuPushFreq(lidar_cmd_set_pcb);
//				get_imu_push_freq();
				livox_gen_cmd_send(START_STOP_SAMP_CMD_ID, lidar_response_pcb, START_SAMPLING);
//				tp_HandleStartStopSampling(lidar_response_pcb);
			}
			/*Here we have to set the "livox_comm_active" as true.*/
			break;
		}
		case QUERY_CMD_ID:
		{
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: Query Response Successful\r\n");
#ifdef TEST_CODE
				return QUERY_SUCCESS;
#endif
			}
			break;
		}
		case HEARTBEAT_CMD_ID:
		{
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
				xil_printf("Success: Heartbeat Response Successful\r\n");

//			/* Send Start Sampling code */
//			tp_HandleStartSampling(lidar_response_pcb);
			break;
		}
		case SAMPLING_CMD_ID:
		{
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: Start Sampling Response Successful\r\n");
#ifdef TEST_CODE
				return SAMPLING_SUCCESS;
#endif
				tp_HandleHeartBeat(lidar_response_pcb);
			}

			break;
		}
		case CH_COORD_CMD_ID:
		{
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: Change Coordinate Response Successful\r\n");
#ifdef TEST_CODE
				return CHANGE_COORD_SUCCESS;
#endif
			}
			break;
		}
		}
	}
	if(resp_cmd_set == 0x01)
	{
		xil_printf("Success: Response is from Lidar command Set\r\n");
		switch(resp_cmd_id)
		{
		case SET_MODE_CMD_ID:
			uint8_t return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: In Setting the Mode\r\n");
				return SET_MODE_SUCCESS;
			}
			else if(return_code == 0x02)
			{
				xil_printf("Success: It's Switching\r\n");
				return SET_MODE_SUCCESS;
			}
			else
			{
				xil_printf("Fail: In setting the Mode\r\n");
			}
		case READ_LIDAR_EXT:

			return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: In Reading the Extrinsic Parameters\r\n");

			}
			memcpy(&read_lidar_ext.roll,&resp_buff[ROLL_IDX],4);
			memcpy(&read_lidar_ext.pitch,&resp_buff[PITCH_IDX],4);
			memcpy(&read_lidar_ext.yaw,&resp_buff[YAW_IDX],4);
			memcpy(&read_lidar_ext.x,&resp_buff[X_IDX],4);
			memcpy(&read_lidar_ext.y,&resp_buff[Y_IDX],4);
			memcpy(&read_lidar_ext.z,&resp_buff[Z_IDX],4);

			return READ_EXT_SUCCESS;

		case WRITE_LID_EXT:
			return_code = resp_buff[RETURN_CODE_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: In Writing the Extrinsic Parameters\r\n");
				return WRITE_EXT_SUCCESS;

			}
			else
			{
				xil_printf("Fail: In Writing the Extrinsic Parameters\r\n");
			}
		case GET_IMU_CMD_ID:
		{
			uint8_t return_code = resp_buff[GET_IMU_RET_IDX];
			uint8_t wht_frequency = resp_buff[FREQUENCY_FETCH_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: In Getting the Response for Get Imu\r\n");
			}
			if(wht_frequency == 0x00)
			{
				xil_printf("Note: It's 0Hz\r\n");
			}
			else
			{
				xil_printf("Note: It's 200Hz\r\n");
			}
		}
		case SET_IMU_CMD_ID:
		{
			uint8_t return_code = resp_buff[SET_IMU_RET_IDX];
			if(return_code == 0x00)
			{
				xil_printf("Success: In Getting response for Set IMU\r\n");
			}
			else
			{
				xil_printf("Failure: In getting response for Set Imu\r\n");
			}

		}

		}
	}
	return;
}
