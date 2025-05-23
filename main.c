/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include "xparameters.h"
#include "netif/xadapter.h"
#include "platform.h"
#include "lidar_data.h"
#include "platform_config.h"
#include "lwipopts.h"
#include "xil_printf.h"
#include "sleep.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "xil_cache.h"
#include "commHandler.h"
#if LWIP_DHCP==1
#include "lwip/dhcp.h"
extern volatile int dhcp_timoutcntr;
#endif

struct netif *netif;
extern uint8_t globalRecDataFlag;
extern uint8_t livox_comm_active;
extern ip_addr_t board_local_ip;
extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;
extern XTtcPs TtcInstance;
extern XScuGic InterruptController;

#define DEFAULT_IP_ADDRESS	"192.168.1.10"
#define DEFAULT_IP_MASK		"255.255.255.0"
#define DEFAULT_GW_ADDRESS	"192.168.1.40"

void platform_enable_interrupts(void);
void start_application(void);
void print_app_header(void);

#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || \
		 XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
int ProgramSi5324(void);
int ProgramSfpPhy(void);
#endif
#endif

#ifdef XPS_BOARD_ZCU102
#ifdef XPAR_XIICPS_0_DEVICE_ID
int IicPhyReset(void);
#endif
#endif

struct netif server_netif;
extern struct udp_pcb *lidar_response_pcb;

 void print_ip(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf("%d.%d.%d.%d\r\n", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

static void print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	print_ip("Board IP:       ", ip);
	print_ip("Netmask :       ", mask);
	print_ip("Gateway :       ", gw);
}

static void assign_default_ip(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	/*-----*/
	int err;

	xil_printf("Configuring default IP %s \r\n", DEFAULT_IP_ADDRESS);

	err = inet_aton(DEFAULT_IP_ADDRESS, ip);
	if (!err)
		xil_printf("Invalid default IP address: %d\r\n", err);

	err = inet_aton(DEFAULT_IP_MASK, mask);
	if (!err)
		xil_printf("Invalid default IP MASK: %d\r\n", err);

	err = inet_aton(DEFAULT_GW_ADDRESS, gw);
	if (!err)
		xil_printf("Invalid default gateway address: %d\r\n", err);
}

int main(void)
{
	struct netif *netif;

	/* the mac address of the board. this should be unique per board */
	unsigned char mac_ethernet_address[] = {
		0x00, 0x0a, 0x35, 0x1d, 0x1e, 0x70 };

	netif = &server_netif;
	IP4_ADDR(&board_local_ip, 192,168,1,10);
	IP4_ADDR(&(netif->ip_addr),  192, 168, 1, 10);
	IP4_ADDR(&(netif->netmask),  255, 255, 255, 0);
	IP4_ADDR(&(netif->gw),  192, 168, 1, 1);
#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || \
		XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
	ProgramSi5324();
	ProgramSfpPhy();
#endif
#endif

	/* Define this board specific macro in order perform PHY reset
	 * on ZCU102
	 */
#ifdef XPS_BOARD_ZCU102
	IicPhyReset();
#endif

	init_platform();

	xil_printf("\r\n\r\n");
	xil_printf("-----lwIP RAW Mode UDP Server Application-----\r\n");

	/* initialize lwIP */
	lwip_init();

	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(netif, &(netif->ip_addr), &(netif->netmask),&(netif->gw), mac_ethernet_address,
				PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\r\n");
		return -1;
	}
	netif_set_default(netif);

	/* now enable interrupts */
	platform_enable_interrupts();

	/* specify that the network if is up */
	netif_set_up(netif);

#if (LWIP_DHCP==1)
	/* Create a new DHCP client for this interface.
	 * Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
	 * the predefined regular intervals after starting the client.
	 */
	dhcp_start(netif);
	dhcp_timoutcntr = 24;
	while (((netif->ip_addr.addr) == 0) && (dhcp_timoutcntr > 0))
		xemacif_input(netif);

	if (dhcp_timoutcntr <= 0) {
		if ((netif->ip_addr.addr) == 0) {
			xil_printf("ERROR: DHCP request timed out\r\n");
			assign_default_ip(&(netif->ip_addr),
					&(netif->netmask), &(netif->gw));
		}
	}

	/* print IP address, netmask and gateway */
#else
	assign_default_ip(&(netif->ip_addr), &(netif->netmask), &(netif->gw));
#endif
	print_ip_settings(&(netif->ip_addr), &(netif->netmask), &(netif->gw));

	xil_printf("\r\n");

	/* print app header */
	print_app_header();

#if 1
	void read_lidar_data_from_ddr4()
	{
		for (int i = 0; i < 3; i++) // Reading 3 sets
		{
			xil_printf("\r\nReading Data from DDR4 - Set %d\r\n", i);

			lidar_data *ddr4_mem = (lidar_data *)(DDR4_BASE_ADDR +
											i * sizeof(lidar_data));

			xil_printf("Lidar ID: %x\r\n", ddr4_mem->lidar_id);
			xil_printf("Status Code: %x%x%x%x\r\n",
					ddr4_mem->status_code[0], ddr4_mem->status_code[1],
					ddr4_mem->status_code[2], ddr4_mem->status_code[3]);
			xil_printf("Timestamp Type: %x\r\n", ddr4_mem->timestamp_type);
			xil_printf("Data Type: %x\r\n", ddr4_mem->data_type);

			for (int j = 0; j < 96; j++)
			{
				xil_printf("PCD[%d]: X=%d, Y=%d, Z=%d, Reflectivity=%d, Tag=%d\r\n",
						j, ddr4_mem->pcd[j].x_axis, ddr4_mem->pcd[j].y_axis,
						ddr4_mem->pcd[j].z_axis, ddr4_mem->pcd[j].reflectivity,
						ddr4_mem->pcd[j].tag);
			}
		}
	}
#endif
	/* start the application*/
#if 1
//	read_lidar_data_from_ddr4();
//	uint8_t name[22] = "BALA SATHYA NARAYANAN";
//	uint8_t status = FfsSdPolledExample(name,sizeof(name));
	start_application();
	xil_printf("\r\n");
	xil_printf("START Application Done\r\n");
	while (1)
	{
		if(livox_comm_active == TRUE)
		{
//			tp_HandleHeartBeat(lidar_response_pcb);
//			sleep(1);
		}
		xemacif_input(netif);
		if(globalRecDataFlag)
		{
			/* Not yet Updated */
		}
	}

	/* never reached */
	cleanup_platform();
#endif
	return 0;
}
