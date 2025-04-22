#include "commHandler.h"


typedef struct sr_car_coord
{
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint8_t relectivity;
	uint8_t tag;
}sr_cart_coord_t;

typedef struct sr_sph_coord
{
	uint32_t depth;
	uint16_t theta;
	uint16_t phi;
	uint8_t reflectivity;
	uint8_t tag;
}sr_sph_coord_t;

typedef struct dual_cart_coord
{
	uint32_t x1;
	uint32_t y1;
	uint32_t z1;
	uint8_t relectivity_1;
	uint8_t tag_1;
	uint32_t x2;
	uint32_t y2;
	uint32_t z2;
	uint8_t relectivity_2;
	uint8_t tag_2;
}dual_cart_coord_t;

/*1.*/int FfsSdPolledExample(uint8_t *data, uint8_t size);

#define PCD_DATA_SIZE 		4
#define ST_CODE_IDX			4
#define TIME_TYPE_IDX		8
#define TIMESTAMP_IDX		10
#define DATA_TYPE_IDX		9
#define PCD_IDX 			18
