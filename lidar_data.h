
#ifndef LIDAR_DATA_H
#define LIDAR_DATA_H

#include "livox_protocol.h"
#include "commHandler.h"
#include <stdint.h>
#include "ip_addr.h"
typedef float float32_t;

#define DDR4_BASE_ADDR	((lidar_data *)0x60001000)
#define DDR_STORAGE		1
#define SDCARD_STORING	0

typedef struct __attribute__((packed)) pcdData
{
	uint8_t x_axis[4];
	uint8_t y_axis[4];
	uint8_t z_axis[4];
	uint8_t reflectivity;
	uint8_t tag;
}pcdData;

typedef struct lidar_data
{
	uint8_t lidar_id;
	uint8_t status_code[4];
	uint8_t timestamp_type;
	uint8_t data_type;
	uint8_t timestamp[8];
	pcdData pcd[100];
}lidar_data;

typedef struct lidar_paramaters
{
	float32_t roll;
	float32_t pitch;
	float32_t yaw;
	uint32_t x;
	uint32_t y;
	uint32_t z;
}lidar_parameters;

/* Macro Variables */
#define VERSION_IDX   0
#define PCD_DATA_SIZE 4
#define PCD_IDX		  18

//static void udp_recv_lidar_data_traffic(void * arg,struct udp_pcb *lidar_pcb,struct pbuf *p,const ip_addr_t *addr, u16_t port);
/* ----------------------------------------
 * 		    Function Prototypes
 *--------------------------------------- */
/*1.*/void fill_initial_data(lidar_data *data,uint8_t * buff);
/*2.*/void lidar_data_processing(uint8_t *buff,uint16_t len);
/*3.*/void configure_lidar_packet(frame_header *fr_hdr,struct pbuf *packetbuf,uint8_t *buff,uint8_t req_len,
								uint8_t CRC32_idx,uint8_t lof,uint8_t seq_num,uint8_t cmd_set,uint8_t cmd_id);
/*4.*/uint8_t set_mode_pkt_send(void);
/*5.*/uint8_t read_lidar_send(void);
/*6.*/uint8_t turn_on_off_suppression(void);
/*7.*/uint8_t write_lidar_send(void);
/*8.*/uint8_t get_imu_push_freq(void);
/*9.*/uint8_t set_imu_push_freq(void);
#endif
