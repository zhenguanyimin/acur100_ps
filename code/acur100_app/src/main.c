
#include "app/app_init.h"
#include <sleep.h>
#include <stdio.h>
#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xil_types.h"
#include "xgpio.h"
int main()
{
//	Xil_Out32(XPAR_AXI2REG3244_0_BASEADDR + 16 , 0x00002780);//ADS W 3244 FCLK 12.5MHz
	app_init();

	while(1);

	return 0;
}
