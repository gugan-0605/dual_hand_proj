#include "commHandler.h"
#include "livox_protocol.h"


/*1.*/void tp_HandlingHandshakeMessage(struct udp_pcb *pcb);
/*2.*/void tp_HandleStartStopSampling(struct udp_pcb *pcb, uint8_t Sampling);
/*3.*/void tp_HandleQueryDevice(struct udp_pcb *pcb);
/*4.*/void tp_HandleChangeCoordinate(struct udp_pcb *pcb);
/*5.*/void tp_HandleDisconnect(struct udp_pcb *pcb);
/*6.*/void tp_HandleHeartBeat(struct udp_pcb *pcb);
