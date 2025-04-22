#ifndef COMM_HANDLER_H
#define COMM_HANDLER_H

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "xil_printf.h"
#include "platform.h"
#include "cmdIndex.h"
#include "xttcps.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "livox_protocol.h"
#include "setting_Buffers.h"
#include "msg_Handling.h"
#include <string.h>

extern const char kLabel[];
extern struct udp_pcb *lidar_cmd;
/* used as indices into kLabel[] */
enum {
	KCONV_UNIT,
	KCONV_KILO,
	KCONV_MEGA,
	KCONV_GIGA,
};

/* used as type of print */
enum measure_t {
	BYTES,
	SPEED
};

/* Report type */
enum report_type {
	/* The Intermediate report */
	INTER_REPORT,
	/* The server side test is done */
	UDP_DONE_SERVER,
	/* Remote side aborted the test */
	UDP_ABORTED_REMOTE
};

struct interim_report {
	u64_t start_time;
	u64_t last_report_time;
	u32_t total_bytes;
	u32_t cnt_datagrams;
	u32_t cnt_dropped_datagrams;
};

struct perf_stats {
	u8_t client_id;
	u64_t start_time;
	u64_t end_time;
	u64_t total_bytes;
	u64_t cnt_datagrams;
	u64_t cnt_dropped_datagrams;
	u32_t cnt_out_of_order_datagrams;
	s32_t expected_datagram_id;
	struct interim_report i_report;
};

/* seconds between periodic bandwidth reports */
#define INTERIM_REPORT_INTERVAL 5
/*------------------------------------------------
 *  			Function Prototypes
 *-----------------------------------------------*/
void tp_HandleLivoxMsg(uint8_t *buf, uint16_t len, struct udp_pcb *pcb);
extern void print_ip(char *msg, ip_addr_t *ip);
extern uint16_t crc16_check(uint8_t *crc_buff);
extern uint32_t crc32_check(uint8_t * crc32_buf);
extern void lidar_cmd_handler_init(void);
extern void lidar_send_gen_cmd(uint8_t command_set, uint8_t command_id);

#endif /* COMM_HANDLER_H*/
