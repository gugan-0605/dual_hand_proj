#ifndef LIVOX_PROTOCOL_H
#define LIVOX_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "commHandler.h"
#include "netif/ethernet.h"
#include "xil_printf.h"


#define SUCCESS 			1
#define XST_SUCCESS			1U
/*-------------------------------------------
 * 			 Global Variables
 *------------------------------------------*/
extern uint8_t local_ip[4];
extern uint16_t livox_data_port;//livox add
extern uint16_t livox_cmd_port;
extern uint16_t livox_imu_port;
/*-------------------------------------------
 * 			 Structure Declarations
 *------------------------------------------*/

typedef struct __attribute__((packed)) frame_header
{
	uint8_t sof;
	uint8_t ver;
	uint16_t len;
	uint8_t cmd_type;
	uint16_t seq_num;
	uint16_t crc16;
	uint32_t crc32;
	uint8_t cmd_set;
	uint8_t cmd_id;
} frame_header;


typedef struct __attribute__((packed))handshake_req
{
	uint8_t user_ip[4];
	uint16_t data_port;
	uint16_t cmd_port;
	uint16_t imu_port;
}handshake_req;
typedef struct gen_cmd_req
{
	uint8_t cmd_set;
	uint8_t cmd_id;
	uint8_t sample_ctrl;
	uint8_t coordinate_type;
	uint32_t status_code;
}gen_cmd_req;
/*-------------------------------------------
 * 			 Function Prototypes
 *------------------------------------------*/
/*0.*/ extern void udp_recv_perf_traffic(void *arg, struct udp_pcb *tpcb,
		struct pbuf *p, const ip_addr_t *addr, u16_t port);
/*1.*/ extern uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint8_t len);
/*2.*/ extern uint32_t crc32(const uint8_t *data, size_t datalen);
/*4.*/ uint8_t set_hs_msg_buffer(uint8_t *hs_buff);
/*5.*/ void lidar_response_handler_init(void);
/*6.*/ uint8_t lidar_response_processing(uint8_t *resp_buff);
/*7.*/ uint8_t set_samp_buff_func(uint8_t *samp_buffer);
/*8.*/ void hs_pkt_send(void);
/*9.*/ uint8_t samp_pkt_send(void);
/*10.*/uint8_t query_pkt_send(void);
/*11.*/uint8_t change_coord_send(void);
/*12.*/uint8_t disconnect_send(void);
/*13.*/void init_frame_header(frame_header *fr_hdr, uint16_t len, uint16_t seq_num, uint8_t cmd_set, uint8_t cmd_id);
/*14.*/void prepare_frame_data(uint8_t *frame_data, frame_header *fr_hdr);
/*15.*/void livox_gen_cmd_send(uint8_t command_id, struct udp_pcb *pcb, uint8_t Samp);

#endif
