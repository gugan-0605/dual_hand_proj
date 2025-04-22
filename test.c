#include "commHandler.h"
#include "livox_protocol.h"
#include "lidar_data.h"
extern uint8_t test_code;
//
//void test_function()
//{
//
//	/*1. Sending the Query packet*/
//	err_t err = query_pkt_send();
//	if(!err)
//		xil_printf("Query Device Packet Sent Successfully\r\n");
//	if(test_code == QUERY_SUCCESS)
//		xil_printf("Query Device Response Successful\r\n");
//	/*After getting the Response we're giving 1 second delay*/
//	sleep(1);
//
//	/*2. Sending the Change Coordinate packet*/
//	err = change_coord_send();
//	if(!err)
//		xil_printf("Change Coordinate Packet sent Successfully\r\n");
//	if(test_code == CHANGE_COORD_SUCCESS)
//		xil_printf("Change Coordinate Response Successful\r\n");
//	/*After getting the Response we're giving 1 second delay*/
//	sleep(1);
//
//	/*3. Sending the Sampling packet*/
//	err = samp_pkt_send();
//	if(!err)
//		xil_printf("Sampling Packet Sent Successfully\r\n");
//	if(test_code == SAMPLING_SUCCESS)
//		xil_printf("Sampling Response Successful\r\n");
//	/*After getting the Response we're giving 1 second delay*/
//	sleep(1);
//
//	/*4. Sending Set Mode Packet*/
//	err = set_mode_pkt_send();
//	if(!err)
//		xil_printf("Set Mode Packet sent Successful\r\n");
//	if(test_code == SET_MODE_SUCCESS)
//		xil_printf("Set Mode response Successful\r\n");
//	/*After getting the Response we're giving 1 second delay*/
//	sleep(1);
//
//	/*5. Read Extrinsic Parameters */
//	err = read_lidar_send();
//	if(!err)
//		xil_printf("Read Extrinsic Packet Sent Successfully\r\n");
//	if(test_code == READ_EXT_SUCCESS)
//		xil_printf("Read Extrinsic Response Successful\r\n");
//	/*After getting the Response we're giving 1 second delay*/
//	sleep(1);
//
//
////	err = disconnect_send();
////	if(!err)
////		xil_printf()
//
//
//}
