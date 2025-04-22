#ifndef CMD_INDEX_H
#define CMD_INDEX_H
/* ------------------------------------
 * 			Ports
 * -----------------------------------*/
#define UDP_CONN_PORT 			55000
#define PTCLOUD_DATA_PORT 		50002
#define GEN_CMD_PORT 			50003
#define LIDAR_CMD_PORT			50004
#define LIVOX_SENSOR_CMD_PORT 	65000
/* ------------------------------------
 * 			Test Plan Macros
 * -----------------------------------*/
#define HS_SUCCESS				1
#define QUERY_SUCCESS 			2
#define SAMPLING_SUCCESS 		3
#define CHANGE_COORD_SUCCESS	4
#define SET_MODE_SUCCESS		5
#define READ_EXT_SUCCESS		6
#define WRITE_EXT_SUCCESS		7
#define DISCONNECT_SUCCESS		8
/* ----------------------------------
 * Frame Index and Header Definitions
 * ----------------------------------*/
#define SOF_IDX 				0
#define SOF  					0XAA;
#define PRO_VERS_IDX			1
#define PRO_VERS				0X01
#define LEN_OF_FRAME_IDX 		2
/* -------------------------------------
 * 			Length of Frames
 * ------------------------------------*/
#define LEN_OF_FRAME_HS			0X19
#define LEN_OF_FRAME_SAMP		0X--

#define CMD_TYPE_IDX			4
/*--------------------------------------
 * 		FRAME SEQUENCE NUMBERS
 *--------------------------------------*/
#define FRAME_SEQ_IDX			5
#define FRAME_SEQ_NUM_HS		0X00
/*---------------------------------------
 * 			CRC Definitions
 *---------------------------------------*/
#define CRC16_IDX				7
#define CRC16_LEN       		7
#define CRC16_SEED 				0x4c49
#define CRC32_SEED 				0x564f580a
/*----------------------------------------
 * 			Command Set and ID's
 *---------------------------------------*/
#define CMD_SET_IDX				9
#define CMD_ID_IDX				10
#define GEN_SEND_CMD_TYPE		0x00
#define GENERAL_CMD_SET			0X00
#define LIDAR_CMD_SET			0X01
#define RETURN_CODE_IDX			11
/* ----------------------------------------
 * 			General Command ID's
 *----------------------------------------*/
#define HANDSHAKE_CMD_ID		0X01
#define QUERY_DEVICE_CMD_ID		0X02
#define HEARTBEAT_CMD_ID		0x03
#define START_STOP_SAMP_CMD_ID	0X04
/* ----------------------------------------
 * 		     Lidar Command Id's
 *----------------------------------------*/
#define SET_MODE				0X00
#define WRITE_LID_EXT			0X01
#define READ_LIDAR_EXT			0X02
#define TURN_ON_OFF_RF			0X03
/*------------------------------------------
 *  	Broadcast Message Related
 *-----------------------------------------*/
#define UNIQUE_ID_SIZE			16
#define	UNIQUE_ID_IDX			11
#define CRC32_IDX_BC			31
#define CRC32_LEN_BC			30
/*------------------------------------------
 *  		Handshake Macros
 *-----------------------------------------*/
#define HANDSHAKE_LEN_OF_FRAME	0x19
#define HANDSHAKE_CMD_TYPE		0X00
#define HANDSHAKE_CMD_SET 		0X00
#define HANDSHAKE_FRAME_SEQ 	0X00
#define HANDSHAKE_PKT_LEN		25
#define HS_IP_ADDR_IDX			11
#define DATA_PORT_IDX			15
#define CMD_PORT_IDX			17
#define IMU_PORT_IDX			19
#define HS_CRC32_IDX			21

/*------------------------------------------
 * 			Heartbeat Macros
 *-----------------------------------------*/

#define HEARTBEAT_REQ_LEN		15
#define HEARTBEAT_LEN_OF_FRAME  0x0F

#define HEARTBEAT_CMD_SET		0x00
#define HEART_CRC32_IDX			11
/*------------------------------------------
 *  		Sampling Macros
 *-----------------------------------------*/
#define SAMPLING_LEN_OF_FRAME	0X10
#define SAMPLING_SEQ_NUM		0x09
#define SAMPLING_REQ_LEN 		16
#define SAMPLING_CTRL_IDX  		11
#define SAMPLING_BUFFER_LEN		0X10
#define SAMPLING_FRAME_SEQ 		0X09
#define SAMPLING_CMD_SET 		0X00
#define SAMPLING_CMD_ID 		0X04
#define START_SAMPLING 			0X01
#define STOP_SAMPLING 			0X00
#define SAMPLING_CRC32_IDX		12
/*------------------------------------------
 *  	Query Device Info Macros
 *-----------------------------------------*/
#define QUERY_REQ_LEN			15
#define QUERY_LEN_OF_FRAME		0X0F
#define QUERY_FRAME_SEQ			0X02
#define QUERY_CMD_SET_IDX		9
#define QUERY_CMD_SET			0X00
#define QUERY_CMD_ID_IDX		10
#define QUERY_CMD_ID			0X02
#define QUERY_CRC_32_IDX		11
/*------------------------------------------
 *  	Change Coordinate Macros
 *-----------------------------------------*/
#define CH_COORD_REQ_LEN		16
#define CH_COORD_LEN_OF_FRAME	0x10
#define CH_COORD_FRAME_SEQ		0X90 //has to change
#define CH_COORD_CMD_ID			0X05
#define CH_COORD_TYPE_IDX		11
#define CH_COORD_CRC32_IDX		12
#define CH_COORD_SPHERICAL		0x01
#define CH_COORD_CARTESIAN		0x00
/*------------------------------------------
 *  		Disconnect Macros
 *-----------------------------------------*/
#define DIS_SEQ					0x00
#define DIS_LOF					0x0F
#define DIS_REQ_LEN				15
#define DIS_CRC32_IDX			11
#define DIS_CMD_ID				0x06
//#define
/* ----------------------------------------
 * 			Lidar Sampling Macros
 *--------------------------------------- */
#define LIDAR_ID_IDX  			2
#define STATUS_CODE_IDX			4
#define TIMESTAMP_TYPE_IDX  	8
#define DATA_TYPE_IDX			9
#define TIMESTAMP_IDX			10
#define DATA_STARTS_IDX			18
/* ----------------------------------------
 * 		    Lidar Set Mode Macros
 *--------------------------------------- */
#define SET_MODE_REQ_LEN		16
#define SET_MODE_CMD_ID			0X00
#define SET_MODE_LOF			0X10
#define SET_MODE_FRAME_SEQ		0X00
#define LIDAR_MODE_IDX			11
#define SET_MODE_CRC32_IDX		12
#define NORMAL_MODE		  		0X01
#define POWER_SAVING_MODE 		0X02
#define STANDBY_MODE 	  		0X03
/* ----------------------------------------
 * 		    Write Extrinsic Macros
 *--------------------------------------- */
#define WRITE_EXT_REQ_LEN 		39
#define WRITE_EXT_LOF			0X27 //Confirm with rajam mam.

#define WRITE_EXT_CRC32_IDX		35
#define WRITE_EXT_CMD_ID		0X01
/* ----------------------------------------
 * 		    Read Extrinsic Macros
 *--------------------------------------- */
#define	READ_EXT_REQ_LEN		15
#define READ_EXT_LOF			0X0F
#define READ_EXT_FRAME_SEQ		0X00
#define READ_EXT_CMD_ID			0X02
#define READ_EXT_CRC32_IDX		11
#define READ_EXT_				0
#define ROLL_IDX				12
#define PITCH_IDX				16
#define YAW_IDX					20
#define X_IDX					24
#define Y_IDX					28
#define Z_IDX					32
/* ----------------------------------------
 * 		 Turn off/on Suppression
 *--------------------------------------- */
#define TURNING_CMD_ID			0x03
#define TURN_ON_STATE			0X01
#define TURNING_STATE_IDX		11
#define TURN_ON_STATE			0X01
#define TURN_OFF_STATE			0X00
#define TURNING_REQ_LEN			16
#define TURNING_LOF				0X10
#define	TURNING_CRC32_IDX		12
#define TURNING_FRAME			0X00
/* ----------------------------------------
 * 		 Set Push IMU Frequency
 *--------------------------------------- */
#define SET_IMU_CMD_ID			0x08
#define FREQUENCY_200			0X01
#define FREQUENCY_0				0x00
#define SET_IMU_REQ_LEN			16
#define FREQUENCY_IDX			11
#define SET_IMU_RET_IDX			11
#define SET_IMU_LOF				0X10
#define SET_IMU_CRC32_IDX		12
/* ----------------------------------------
 * 		 Get Push IMU Frequency
 *--------------------------------------- */
#define GET_IMU_CMD_ID			0x09
#define FREQUENCY_FETCH_IDX		12
#define GET_IMU_RET_IDX			11
#define GET_IMU_REQ_LEN			15
#define GET_IMU_LOF				0X0F
#define GET_IMU_CRC32_IDX		11

#endif
