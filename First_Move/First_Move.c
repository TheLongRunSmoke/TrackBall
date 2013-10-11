
#include "Mouse.h"

/** Indicates what report mode the host has requested, true for normal HID reporting mode, \c false for special boot
 *  protocol reporting mode.
 */
static bool UsingReportProtocol = true;

/** Current Idle period. This is set by the host via a Set Idle HID class request to silence the device's reports
 *  for either the entire idle duration, or until the report status changes (e.g. the user moves the mouse).
 */
static uint16_t IdleCount = 0;

/** Current Idle period remaining. When the IdleCount value is set, this tracks the remaining number of idle
 *  milliseconds. This is separate to the IdleCount timer and is incremented and compared as the host may request
 *  the current idle period via a Get Idle HID class request, thus its value must be preserved.
 */
static uint16_t IdleMSRemaining = 0;

static uint8_t PositionChange = 0;
static uint8_t LastStateX = 0;
static uint8_t LastStateY = 0;

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	for (;;)
	{
		Grey_Task();
		Mouse_Task();
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	
	/* Hardware Initialization */
	LEDs_Init();
	Buttons_Init();
	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{

	/* Default to report protocol on connect */
	UsingReportProtocol = true;
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and Mouse reporting tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the mouse reporting task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoint */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(MOUSE_EPADDR, EP_TYPE_INTERRUPT, MOUSE_EPSIZE, 1);

	/* Turn on Start-of-Frame events for tracking HID report period expiry */
	USB_Device_EnableSOFEvents();

}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_MouseReport_Data_t MouseReportData;

				/* Create the next mouse report for transmission to the host */
				CreateMouseReport(&MouseReportData);

				Endpoint_ClearSETUP();

				/* Write the report data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&MouseReportData, sizeof(MouseReportData));
				Endpoint_ClearOUT();

				/* Clear the report data afterwards */
				memset(&MouseReportData, 0, sizeof(MouseReportData));
			}

			break;
		case HID_REQ_GetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current protocol flag to the host */
				Endpoint_Write_8(UsingReportProtocol);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
		case HID_REQ_SetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Set or clear the flag depending on what the host indicates that the current Protocol should be */
				UsingReportProtocol = (USB_ControlRequest.wValue != 0);
			}

			break;
		case HID_REQ_SetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Get idle period in MSB, must multiply by 4 to get the duration in milliseconds */
				IdleCount = ((USB_ControlRequest.wValue & 0xFF00) >> 6);
			}

			break;
		case HID_REQ_GetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current idle duration to the host, must be divided by 4 before sent to host */
				Endpoint_Write_8(IdleCount >> 2);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
	}
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	/* One millisecond has elapsed, decrement the idle time remaining counter if it has not already elapsed */
	if (IdleMSRemaining)
	  IdleMSRemaining--;
}

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] ReportData  Pointer to a HID report data structure to be filled
 */
void CreateMouseReport(USB_MouseReport_Data_t* const ReportData)
{
	uint8_t ButtonStatus_LCL = Buttons_GetStatus();

	/* Clear the report contents */
	memset(ReportData, 0, sizeof(USB_MouseReport_Data_t));

	if (PositionChange & ((1<<0)|(1<<1)))
	{
		if (PositionChange & (1<<0)) ReportData->X =  5;
		if (PositionChange & (1<<1)) ReportData->X = -5;
	} 
	if (PositionChange & ((1<<2)|(1<<3)))
	{
		if (PositionChange & (1<<2)) ReportData->Y =  5;
		if (PositionChange & (1<<3)) ReportData->Y = -5;
	}
	
	PositionChange = 0;

	if (ButtonStatus_LCL & BUTTONS_BUTTON1)
	  ReportData->Button |= (1 << 0);
}

/** Sends the next HID report to the host, via the keyboard data endpoint. */
void SendNextReport(void)
{
	static USB_MouseReport_Data_t PrevMouseReportData;
	USB_MouseReport_Data_t        MouseReportData;
	bool                          SendReport;

	/* Create the next mouse report for transmission to the host */
	CreateMouseReport(&MouseReportData);

	/* Check to see if the report data has changed - if so a report MUST be sent */
	SendReport = (memcmp(&PrevMouseReportData, &MouseReportData, sizeof(USB_MouseReport_Data_t)) != 0);

	/* Check if the idle period is set and has elapsed */
	if (IdleCount && (!(IdleMSRemaining)))
	{
		/* Reset the idle time remaining counter */
		IdleMSRemaining = IdleCount;

		/* Idle period is set and has elapsed, must send a report to the host */
		SendReport = true;
	}

	/* Select the Mouse Report Endpoint */
	Endpoint_SelectEndpoint(MOUSE_EPADDR);

	/* Check if Mouse Endpoint Ready for Read/Write and if we should send a new report */
	if (Endpoint_IsReadWriteAllowed() && SendReport)
	{
		/* Save the current report data for later comparison to check for changes */
		PrevMouseReportData = MouseReportData;

		/* Write Mouse Report Data */
		Endpoint_Write_Stream_LE(&MouseReportData, sizeof(MouseReportData), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();
	}
}

/** Task to manage HID report generation and transmission to the host, when in report mode. */
void Mouse_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Send the next mouse report to the host */
	SendNextReport();
}

/** Task for decode grey code from trackball */
void Grey_Task(void)
{
	uint8_t NewStateX = 0;
	uint8_t NewStateY = 0;
	
	NewStateX = PIND & 0x14;  //Накладываю маску 00010100
	NewStateY = PIND & 0x0A;  //Накладываю маску 00001010
	
	switch (LastStateX)
		{
			case 0x00:
			{
				if (NewStateX == 0x04) PositionChange |= (1<<0);		//Увеличиваю X
				if (NewStateX == 0x10) PositionChange |= (1<<1);		//Уменьшаю X
				break;
			}
			case 0x04:
			{
				if (NewStateX == 0x14) PositionChange |= (1<<0);		//Увеличиваю X
				if (NewStateX == 0x00) PositionChange |= (1<<1);		//Уменьшаю X
				break;
			}
			case 0x14:
			{
				if (NewStateX == 0x10) PositionChange |= (1<<0);		//Увеличиваю X
				if (NewStateX == 0x04) PositionChange |= (1<<1);		//Уменьшаю X
				break;
			}
			case 0x10:
			{
				if (NewStateX == 0x00) PositionChange |= (1<<0);		//Увеличиваю X
				if (NewStateX == 0x14) PositionChange |= (1<<1);		//Уменьшаю X
				break;
			}
		}
		
		switch (LastStateY)
		{
			case 0x00:
			{
				if (NewStateY == 0x02) PositionChange |= (1<<2);		//Увеличиваю Y
				if (NewStateY == 0x08) PositionChange |= (1<<3);		//Уменьшаю Y
				break;
			}
			case 0x02:
			{
				if (NewStateY == 0x0A) PositionChange |= (1<<2);		//Увеличиваю Y
				if (NewStateY == 0x00) PositionChange |= (1<<3);		//Уменьшаю Y
				break;
			}
			case 0x0A:
			{
				if (NewStateY == 0x08) PositionChange |= (1<<2);		//Увеличиваю Y
				if (NewStateY == 0x02) PositionChange |= (1<<3);		//Уменьшаю Y
				break;
			}
			case 0x08:
			{
				if (NewStateY == 0x00) PositionChange |= (1<<2);		//Увеличиваю Y
				if (NewStateY == 0x0A) PositionChange |= (1<<3);		//Уменьшаю Y
				break;
			}
		}
	
	LastStateX = NewStateX;
	LastStateY = NewStateY;
}