
#include "app_init.h"
#include "xtime_l.h"
#include <stdio.h>
#include "xparameters.h"
#include "netif/xadapter.h"
#include "../cfg/platform_config.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "../srv/log/log.h"
#include "../srv/cli/cli_if.h"
#include "../srv/protocol/protocol_if.h"
#include "../test/ut/unit_test.h"
#include "../drv/awmf_0165/awmf_0165_drv.h"
#include "../hal/LMX2492Module/LMX2492_config/lmx2492_config_register.h"
#include "../app/Wave_Configuration/wave_config.h"
#include "../app/Tracking/tracking_interface.h"
#include "../app/Detection/detection_interface.h"
#include "DataPath/data_path.h"
#include "../drv/int_handle/interrupt_handle.h"

#include "xgpiops.h"
#include "sleep.h"


#if LWIP_IPV6==1
#include "lwip/ip.h"
#else
#if LWIP_DHCP==1
#include "lwip/dhcp.h"
#endif
#endif

#if LWIP_IPV6==0
#if LWIP_DHCP==1
extern volatile int dhcp_timoutcntr;
err_t dhcp_start(struct netif *netif);
#endif
#endif

#define THREAD_STACKSIZE 1024

static struct netif server_netif;


protocol_object_list_detected_t gDetectList[1] = { 0 };
protocol_object_list_tracked_t gTrackList[1] = { 0 };
protocol_beam_scheduling_t gBeamInfo[1] = { 0 };
protocol_cfg_param_t gConfigParmInfo[1] = { 0 };
protocol_radar_platfrom_info_t gPlatformInfo[1] = { 0 };
protocol_radar_status_t gRadarStatus[1] = { 0 };


#if LWIP_IPV6==1
void print_ip6(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf(" %x:%x:%x:%x:%x:%x:%x:%x\n\r",
		IP6_ADDR_BLOCK1(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK2(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK3(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK4(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK5(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK6(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK7(&ip->u_addr.ip6),
		IP6_ADDR_BLOCK8(&ip->u_addr.ip6));
}
#else
void print_ip(char *msg, ip_addr_t *ip)
{
	xil_printf(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
		ip4_addr3(ip), ip4_addr4(ip));
}

void print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	print_ip("Board IP: ", ip);
	print_ip("Netmask : ", mask);
	print_ip("Gateway : ", gw);
}
#endif

#if SET_ETH_PHY_IO_AT_STARTUP
static void net_phy_reset_func(void)
{
	s32 xStatus = 0;
	XGpioPs gpioInsPs = {0};
	XGpioPs_Config *pGpioConfig = NULL;

	pGpioConfig = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
	if (pGpioConfig == NULL)
	{
		return;
	}

	xStatus = XGpioPs_CfgInitialize(&gpioInsPs, pGpioConfig, pGpioConfig->BaseAddr);
	if (xStatus != XST_SUCCESS)
	{
		return;
	}

	XGpioPs_SetDirectionPin(&gpioInsPs, 51, 1);
	(void)XGpioPs_SetOutputEnablePin(&gpioInsPs, 51, 1);

	XGpioPs_WritePin(&gpioInsPs, 51, 1);
	XGpioPs_WritePin(&gpioInsPs, 51, 0);
	XGpioPs_WritePin(&gpioInsPs, 51, 1);

	return;
}
#endif /* SET_ETH_PHY_IO_AT_STARTUP */

void network_thread(void *p)
{
    struct netif *netif;
    /* the mac address of the board. this should be unique per board */
    unsigned char mac_ethernet_address[] = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x02};
#if LWIP_IPV6==0
    ip_addr_t ipaddr, netmask, gw;
#if LWIP_DHCP==1
    int mscnt = 0;
#endif
#endif

    netif = &server_netif;

    xil_printf("\r\n\r\n");
    xil_printf("-----lwIP Socket Mode ------\r\n");

    vTaskDelay(3 * pdMS_TO_TICKS(1000));
#if SET_ETH_PHY_IO_AT_STARTUP
    /* Force reset PHY after initialization */
    net_phy_reset_func();
#endif /* SET_ETH_PHY_IO_AT_STARTUP */

#if LWIP_IPV6==0
#if LWIP_DHCP==0
    /* initialize IP addresses to be used */
    // ref IP_ADDR_LOCAL
    IP4_ADDR(&ipaddr,  192, 168, 235, 55);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw,      192, 168, 235, 1);
#endif

#if LWIP_DHCP==0
    /* print out IP settings of the board */
    print_ip_settings(&ipaddr, &netmask, &gw);
#endif

#if LWIP_DHCP==1
	ipaddr.addr = 0;
	gw.addr = 0;
	netmask.addr = 0;
#endif
#endif

#if LWIP_IPV6==0
    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add(netif, &ipaddr, &netmask, &gw, mac_ethernet_address, PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\r\n");
		return;
    }
#else
    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add(netif, NULL, NULL, NULL, mac_ethernet_address, PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\r\n");
		return;
    }

    netif->ip6_autoconfig_enabled = 1;

    netif_create_ip6_linklocal_address(netif, 1);
    netif_ip6_addr_set_state(netif, 0, IP6_ADDR_VALID);

    print_ip6("\n\rBoard IPv6 address ", &netif->ip6_addr[0].u_addr.ip6);
#endif

    netif_set_default(netif);

    /* specify that the network if is up */
    netif_set_up(netif);

    /* start packet receive thread - required for lwIP operation */
    (void)sys_thread_new("xemacif_input_thread", (void(*)(void*))xemacif_input_thread, netif, TASK_STACK_SIZE_ETH_RCV, TASK_PRI_ETH_RCV);

#if LWIP_IPV6==0
#if LWIP_DHCP==1
    dhcp_start(netif);
    while (1) {
		vTaskDelay(DHCP_FINE_TIMER_MSECS / portTICK_RATE_MS);
		dhcp_fine_tmr();
		mscnt += DHCP_FINE_TIMER_MSECS;
		if (mscnt >= DHCP_COARSE_TIMER_SECS*1000) {
			dhcp_coarse_tmr();
			mscnt = 0;
		}
	}
#else
    xil_printf("\r\n");

    (void)sys_thread_new("ALG_THRD", radar_alg_task, 0, TASK_STACK_SIZE_ALG_TRACKING * 4, TASK_PRI_ALG_TRACKING);
    data_path_init();
    DetectAlgInit();

    /* initialize interrupt functions */
    IntcInitFunction(INTC_DEVICE_ID);

//    data_path_start(ADC_DATA);
//    data_path_out_en(ADC_DATA);
//    data_path_start(RDMAP_DATA);
//    data_path_out_en(RDMAP_DATA);

    vTaskDelete(NULL);
#endif
#else
    xil_printf("\r\n");

    vTaskDelete(NULL);
#endif
    return;
}

void cli_init(void)
{
	vRegisterSampleCLICommands();

	vUARTCommandConsoleStart(TASK_STACK_SIZE_CLI_SERVER, TASK_PRI_CLI_SERVER);

	(void)xTaskCreate(vBasicSocketsCommandInterpreterTask, "TCP_CLI", TASK_STACK_SIZE_CLI_SERVER, NULL, TASK_PRI_CLI_SERVER, NULL);
	// (void)sys_thread_new("TCP_CLI", vBasicSocketsCommandInterpreterTask, NULL, TASK_STACK_SIZE_CLI_SERVER, TASK_PRI_CLI_SERVER);

	return;
}

void radar_alg_task(void *p)
{
	s32 ret = -1 ;

	while(1)
	{
		ret = runDetectAlgBlocking();
		if (-1 == ret)
		{
			vTaskDelay(1);
			continue;
		}
		runTrackingAlg();
	}
}

void init_thread(void *p)
{
#if LWIP_DHCP==1
	int mscnt = 0;
#endif

	/* initialize lwIP before calling sys_thread_new */
	lwip_init();

	(void)sys_thread_new("NW_THRD", network_thread, NULL, TASK_STACK_SIZE_NETWORK_CFG, TASK_PRI_NETWORK_CFG);

	/* initialize and open hardwares */
    VCOPLLInitFunction();
	(void)awmf_0165_drv_init();
    // (void)awmf_0165_drv_open();

    /* set log level for debug purpose */
    log_level_t log_level_num_old;
    const char *log_level_str_old;
    log_level_num_old = log_get_lvl_num();
    log_level_str_old = log_get_lvl_str();
    log_set_lvl(LL_ERROR);
    xil_printf("log level changed from %d (%s) to %d (%s)\r\n",
    log_level_num_old, log_level_str_old, log_get_lvl_num(), log_get_lvl_str());

    /* initialize service software components */
    cli_init();

    (void)protocol_init();

    /* initialize unit test component */
    uint_test_init();

#if LWIP_IPV6==0
#if LWIP_DHCP==1
    while (1) {
		vTaskDelay(DHCP_FINE_TIMER_MSECS / portTICK_RATE_MS);
		if (server_netif.ip_addr.addr) {
			xil_printf("DHCP request success\r\n");
			print_ip_settings(&(server_netif.ip_addr), &(server_netif.netmask), &(server_netif.gw));
			xil_printf("\r\n");
			break;
		}
		mscnt += DHCP_FINE_TIMER_MSECS;
		if (mscnt >= DHCP_COARSE_TIMER_SECS * 2000) {
			xil_printf("ERROR: DHCP request timed out\r\n");
			xil_printf("Configuring default IP of 192.168.1.10\r\n");
			IP4_ADDR(&(server_netif.ip_addr),  192, 168, 1, 10);
			IP4_ADDR(&(server_netif.netmask), 255, 255, 255,  0);
			IP4_ADDR(&(server_netif.gw),  192, 168, 1, 1);
			print_ip_settings(&(server_netif.ip_addr), &(server_netif.netmask), &(server_netif.gw));
			/* print all application headers */
			xil_printf("\r\n");
			xil_printf("%20s %6s %s\r\n", "Server", "Port", "Connect With..");
			xil_printf("%20s %6s %s\r\n", "--------------------", "------", "--------------------");
			break;
		}
	}
#endif
#endif

    vTaskDelete(NULL);
    return;
}

void app_init(void)
{
    /* any thread using lwIP should be created using sys_thread_new */
	(void)sys_thread_new("INIT_THRD", init_thread, 0, TASK_STACK_SIZE_APP_INIT, TASK_PRI_APP_INIT);

	vTaskStartScheduler();

	return;
}
