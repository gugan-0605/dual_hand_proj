#include "commHandler.h"
XTtcPs TtcInstance;
XScuGic InterruptController;
extern struct udp_pcb *lidar_response_pcb;
extern uint8_t livox_comm_active;

void TimerInterruptHandler(void *CallBackRef)
{
	XTtcPs *TtcPtr = (XTtcPs *) CallBackRef;

	// Clear Timer Interrupt
	XTtcPs_ClearInterruptStatus(TtcPtr, XTTCPS_IXR_INTERVAL_MASK);

	xil_printf("TTC Interrupt Triggered\r\n");
	// Send Heartbeat ONLY if livox_comm_active is TRUE
	if (livox_comm_active)
	{
		tp_HandleHeartBeat(lidar_response_pcb);
		xil_printf("Heartbeat Sent!\r\n");
	}
}

void SetupTTC(XTtcPs *TtcInstancePtr)
{
	XTtcPs_Config *ConfigPtr;
	u16 Interval;
	u8 Prescaler;

	// Get TTC Configuration
	ConfigPtr = XTtcPs_LookupConfig(XPAR_XTTCPS_0_DEVICE_ID);
	XTtcPs_CfgInitialize(TtcInstancePtr, ConfigPtr, ConfigPtr->BaseAddress);

	// Calculate Interval & Prescaler for 1 Hz
	XTtcPs_CalcIntervalFromFreq(TtcInstancePtr, 1, &Interval, &Prescaler);
	XTtcPs_SetInterval(TtcInstancePtr, Interval);
	XTtcPs_SetPrescaler(TtcInstancePtr, Prescaler);

	// Enable Interval Mode
	XTtcPs_SetOptions(TtcInstancePtr, XTTCPS_OPTION_INTERVAL_MODE);
}

void SetupInterruptSystem(XScuGic *InterruptControllerPtr, XTtcPs *TtcInstancePtr)
{
	XScuGic_Config *IntcConfig;

	// Initialize Interrupt Controller
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	XScuGic_CfgInitialize(InterruptControllerPtr, IntcConfig, IntcConfig->CpuBaseAddress);

	// Connect TTC Timer Interrupt
	XScuGic_Connect(InterruptControllerPtr, XPAR_XTTCPS_0_INTR,
					(Xil_ExceptionHandler) TimerInterruptHandler,
					(void *) TtcInstancePtr);

	// Enable TTC Interrupt in GIC
	XScuGic_Enable(InterruptControllerPtr, XPAR_XTTCPS_0_INTR);

	// Enable Interrupts in Processor
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,(Xil_ExceptionHandler) XScuGic_InterruptHandler,
			 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  InterruptControllerPtr);
	Xil_ExceptionEnable();
}


