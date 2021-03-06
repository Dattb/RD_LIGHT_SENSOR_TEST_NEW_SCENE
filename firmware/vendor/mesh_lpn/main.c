/********************************************************************************************************
 * @file     main.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "vendor/common/mesh_lpn.h"
#include "proj/drivers/i2c.h"
#include "vendor/common/mesh_lpn.h"
#include "vendor/Rang_Dong/RD_Light_Sensor.h"
#include "vendor/Rang_Dong/Sensor_Flash.h"
extern void user_init();
extern void main_loop ();

#if (HCI_ACCESS==HCI_USE_UART)
#include "proj/drivers/uart.h"
extern my_fifo_t hci_rx_fifo;
u8 count_scene=0;
u8 flag_2_sleep=0;
extern u8 Sence_call_buff [];
extern RD_SensorSencesStoreTypedef RD_Sence_Store_obj[];
u16 uart_tx_irq=0, uart_rx_irq=0;

_attribute_ram_code_ void irq_uart_handle()
{
	unsigned char irqS = reg_dma_rx_rdy0;
	if(irqS & FLD_DMA_CHN_UART_RX)	//rx
	{
		uart_rx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_RX;
		u8* w = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
		if(w[0]!=0)
		{
			my_fifo_next(&hci_rx_fifo);
			u8* p = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
			reg_dma0_addr = (u16)((u32)p);
		}
	}

	if(irqS & FLD_DMA_CHN_UART_TX)	//tx
	{
		uart_tx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_TX;
	}
}
#endif

_attribute_ram_code_ void irq_handler(void)
{
	#if DUAL_MODE_ADAPT_EN
	if(rf_mode == RF_MODE_ZIGBEE){
		irq_zigbee_sdk_handler();
	}else
	#endif
	{
    	lpn_debug_set_irq_pin(1);
		irq_blt_sdk_handler ();  //ble irq proc
    	lpn_debug_set_irq_pin(0);
	}

#if (HCI_ACCESS==HCI_USE_UART)
	irq_uart_handle();
#endif
}

FLASH_ADDRESS_DEFINE;
#if(MCU_CORE_TYPE == MCU_CORE_8269)
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	clock_init();
	set_tick_per_us(CLOCK_SYS_CLOCK_HZ/1000000);

	gpio_init();

	rf_drv_init(CRYSTAL_TYPE);

	user_init ();

    irq_enable();

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
_attribute_ram_code_ int main (void)    //must run in ramcode
{
	FLASH_ADDRESS_CONFIG;
	blc_pm_select_internal_32k_crystal();
#if(MCU_CORE_TYPE == MCU_CORE_8258)
	cpu_wakeup_init();
#elif(MCU_CORE_TYPE == MCU_CORE_8278)
	cpu_wakeup_init(LDO_MODE,EXTERNAL_XTAL_24M);
#endif

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_drv_init(RF_MODE_BLE_1M);

	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	clock_init(SYS_CLK_16M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	clock_init(SYS_CLK_24M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	clock_init(SYS_CLK_32M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	clock_init(SYS_CLK_48M_Crystal);
#endif


#if	(PM_DEEPSLEEP_RETENTION_ENABLE)
		if( pm_is_MCU_deepRetentionWakeup() ){
			user_init_deepRetn ();
		}
		else
#endif
	{
		user_init();
	}

		flag_2_sleep=1;
    irq_enable();
    //uart_CSend("check uart here");
	#if (MESH_USER_DEFINE_MODE == MESH_IRONMAN_MENLO_ENABLE)
	LOG_USER_MSG_INFO(0, 0,"[mesh] Start from SIG Mesh", 0);
	#endif
	//uart_CSend("day la dai tieng noi Viet Nam\r\n");
//	sleep_us(10000);
	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
//		//			count_time++;
//					if(count_scene <= 5)
//					{
//						RD_Send_Lux(count_scene);
////						for(u8 i=0;i<NUMBER_OF_SCENE;i++){
////							if (Sence_call_buff[i]){
////								u16 SceneID = (RD_Sence_Store_obj[i].SceneID[1]<<8)|RD_Sence_Store_obj[i].SceneID[0];
////								u16 SrgbID = RD_Sence_Store_obj[i].SrgbID;
////								access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
////								if(SrgbID)RD_SensorSendRGBScene(SceneID);
////
////							}
////							Sence_call_buff[i]=0;
////						}
//						count_scene++;
//
//					}
//					else{
//						flag_2_sleep=0;
//					}


		main_loop ();
	}
}
#endif
