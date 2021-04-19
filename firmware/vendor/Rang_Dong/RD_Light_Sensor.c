/*
 * RD_Light_Sensor.c
 *
 *  Created on: Nov 19, 2020
 *      Author: Dat UTC
 */


#include "RD_Light_Sensor.h"
u8 check_done=0;
u8 check_poll=1;
u16 lux_val_old,lux_val_new;
u8 poll_time_out = 65;  // 65s
u16 friend_poll =4000;
u16 provision_time=60000;
bool lux_changle;
u16 RD_LS_TransferCntInMin = 60*1000;    // 60*1000=60.000 ms
u32 RD_LS_TransferCntInHour = 60*60*1000;   // 60*60*1000 =3.600.000 ms



u8 Sence_call_buff[NUMBER_OF_SCENE];



u8 sensor_to_gw_tx_buff[GW_TX_BUFF_DATA_LEN]={0};
u8 power_to_gw_tx_buff[GW_TX_BUFF_DATA_LEN]={0};
u8 i2c_tx_buff[TX_BUFF_DATA_LEN] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
u8 i2c_rx_buff[RX_BUFF_DATA_LEN] = {0};
u8 RD_Send_Scene_to_RGB_Buff[GW_TX_BUFF_DATA_LEN]={0};

u16 Gw_addr=0x0001;



extern RD_SensorSencesStoreTypedef RD_Sence_Store_obj[];
extern u8  RD_Sence_Flash_Read_Buff [];
extern u8  RD_Sence_Flash_Write_Buff[];
bool chek_3_cycle_flag;
u16 SenceID;
/*
 * RD_power_read()
 * Ham co tac dung doc ra ADC cho biet dien ap cua pin qua do dua vao bo tham so da khao sat se biet duoc
 * ung voi dien ap bao nhieu thi dung luong pin con lai la bao nhieu.
 */
unsigned int RD_power_read()
{

    RD_ADC_init (GPIO_PC4);
	unsigned int power_read;
	float power_persent=0;
    //sleep_us(2000);
	power_read = adc_sample_and_get_result();
	if(power_read<=1070)
	{
		power_persent=  power_read*0.0513-45.82;   // %pin = vol*0.0513-45.82
	}
	else if (power_read<=1174)
	{
		power_persent=  power_read*0.2449-254.62; //y = 0.2449x - 254.62
	}
	else if (power_read<=1404)
	{
		power_persent=  power_read*0.3084-333.94;          //y = 0.3084x - 333.94
	}
	else if(power_read<1480) {
		power_persent=  power_read*0.1273-89.77;  	//y = 0.1273x - 89.77
	}
	else power_persent=100;
	power_persent = (unsigned int) power_persent;
	return power_persent;
//	return power_read;
}

/*
 * RD_ADC_init (u32 gpio)
 * Ham khoi tao bo ADC day la cau hinh khuyen dung
 */
void RD_ADC_init (u32 gpio)
{
	adc_init();
	adc_base_init(gpio);
	adc_power_on_sar_adc(1);

}
/*
 * RD_light_sensor_tx (u16 loop)
 * Day la ham tinh toan va truyen cuong do sang cua cam bien.
 * Dua vao thoi gian cho vong loop co the cai dat duoc sau bao lau cam bien moi gui
 * ban tin bao cao cuong do sang 1 lan
 * khi an nut se gui ca cuong do sang va % pin
 */
void RD_light_sensor_tx (u16 loop)
{

	if(!check_done)
	{
		*sensor_to_gw_tx_buff = LIGHT_SENSOR_MODULE_TYPE;
		*(sensor_to_gw_tx_buff+1) = (u8)(LIGHT_SENSOR_MODULE_TYPE << 8);

		*power_to_gw_tx_buff = SENSOR_POWER_TYPE;
		*(power_to_gw_tx_buff+1) = (u8)(SENSOR_POWER_TYPE << 8);
	    check_done=1;
	}
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
	i2c_write_series(OPT3001_CONFIG_REGISTER,OPT3001_CONFIG_REGISTER_LEN,(u8 *)i2c_tx_buff, TX_BUFF_DATA_LEN);
	i2c_read_series(OPT3001_RESULT_REGISTER,OPT3001_RESULT_REGISTER_LEN, (u8 *)i2c_rx_buff, RX_BUFF_DATA_LEN);

	lux_val_new = (i2c_rx_buff[0]<<8) | i2c_rx_buff[1];
	if(lux_val_new!=lux_val_old)
	{
		if(CalculateLux(lux_val_new)>=CalculateLux(lux_val_old))
		{
			if(CalculateLux(lux_val_new)-CalculateLux(lux_val_old)>=20)
			{
				lux_val_old=lux_val_new;
				lux_changle=TRUE;
			}
		}
		else if(CalculateLux(lux_val_new)<=CalculateLux(lux_val_old))
		{
			if(CalculateLux(lux_val_old)-CalculateLux(lux_val_new)>=20)
			{
				lux_val_old=lux_val_new;
				lux_changle=TRUE;
			}
		}
	}
	if(check_poll>=loop||!gpio_read(GPIO_PD4)||lux_changle==TRUE)
	{
		RD_Send_Lux(0x0000);
		if(check_poll>=loop||!gpio_read(GPIO_PD4)) RD_Send_LightSensorPower();
		check_poll=0;

		/*
		 * TODO: truong data anh sang khi gui di duoc du nguyen ma khong bi dao bit
		 */
	}
	 check_poll++;
	 if(chek_3_cycle_flag==0){
		 if(check_poll%3==0)
		 chek_3_cycle_flag =1;
	 }

	 RD_LightSensorControlSence(lux_val_new);

	// RD_Send_Lux(0x0000);
}
/*
 * Ham truc tiep gui gia tri cuong do sang
 */
void RD_Send_Lux(u16 SceneID)
{
	*(sensor_to_gw_tx_buff+2) = * i2c_rx_buff;
	*(sensor_to_gw_tx_buff+3) = *(i2c_rx_buff+1);
	*(sensor_to_gw_tx_buff+4) =(u8)(SceneID);
	*(sensor_to_gw_tx_buff+5) =(u8)(SceneID>>8);

	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)sensor_to_gw_tx_buff, GW_TX_BUFF_DATA_LEN, GATEWAY_ADDR, RSP_MAX);
}
/*
 * Ham truc tiep gui gia tri % pin cua cam bien
 */
void RD_Send_LightSensorPower()
{
	u16 Power_Data=RD_power_read();
	*(power_to_gw_tx_buff+2) = (u8) (Power_Data>>8);
	*(power_to_gw_tx_buff+3) = (u8)(Power_Data);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)power_to_gw_tx_buff, GW_TX_BUFF_DATA_LEN, GATEWAY_ADDR, RSP_MAX);
}
/*
 * Khai bao led cua cam bien
 */
void RD_Led_init(void)
{
		gpio_set_func(GPIO_PB4 ,AS_GPIO);
		gpio_set_output_en(GPIO_PB4, 1); 		//enable output
		//gpio_set_input_en(GPIO_PB4 ,0);			//disable input
		gpio_setup_up_down_resistor(GPIO_PB4,PM_PIN_PULLUP_1M);
}
/*
 * Ham delay
 */
void delay_init(u16 time)
{
	while (time--);
}
/*
 * Ham co tac dung thay doi poll time cua cam bien.
 * Ham se nhan tham so *par trong ban tin time poll change de thay doi thoi gian poll cua cam bien
 */
void time_poll_change (u8 *par,mesh_cb_fun_par_t *cb_par)
{
	if(cb_par->op==0x3082)
	{
		unsigned int data;
		data = (par[0]<<8) | par[1];
		friend_poll = data;
		sleep_us(2000); // delay de gui uart
	}

}

/*
 *  CalculateLux(unsigned int rsp_lux)
 *  Ham se tinh toan ra gia tri cuong do sang dua vao data nhan tu I2C
 */
unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	//Lux_Value = 0.01 * pow(2,Lux_MSB) * Lux_LSB; //don't use
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value=0.01 * pow * lux_LSB;
	return lux_Value;
}



void RD_SensorSendRGBScene(u16 app_rgbid){
	RD_Send_Scene_to_RGB_Buff[0] = (u8)( RD_HEADER_SENSOR_SEND_RGB_SCENE);
	RD_Send_Scene_to_RGB_Buff[1] = (u8)( RD_HEADER_SENSOR_SEND_RGB_SCENE>>8);
	RD_Send_Scene_to_RGB_Buff[2] = (u8)( app_rgbid);
	RD_Send_Scene_to_RGB_Buff[3] = (u8)( app_rgbid>>8);
	mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_SEND, (u8 *)RD_Send_Scene_to_RGB_Buff, GW_TX_BUFF_DATA_LEN, 0xffff, RSP_MAX);
}

/*
 * RD_LightSensorControlSence(u16 Sensor_Lux)
 * Ham se dieu khien scene dua vao ConditionType cua cac obj de cho biet kieu dieu kien la >,< hay ><
 * roi dua vao cuong do sang cua cua cac obj so sanh voi cuong do sang tra ve tu I2C de dieu khien
 */

void RD_LightSensorControlSence(u16 Sensor_Lux)
{
	//if(lux_changle){
		Sensor_Lux = CalculateLux(Sensor_Lux);
		// Cho nay xu ly data nhan ve tu gateway, tach ra nhung phan data can su dung
		for(u8 i=0;i<NUMBER_OF_SCENE;i++){

//			blt_sdk_main_loop ();
//			proc_ui();
//			proc_led();
//			mesh_loop_process();

			u16 SceneID = (RD_Sence_Store_obj[i].SceneID[1]<<8)|RD_Sence_Store_obj[i].SceneID[0];
			u8 Light_Condition = RD_Sence_Store_obj[i].Light_Condition;
			u16 LuxLow = (RD_Sence_Store_obj[i].LuxLow[1]<<8)|RD_Sence_Store_obj[i].LuxLow[0];
			u16 LuxHight = (RD_Sence_Store_obj[i].LuxHight[1]<<8)|RD_Sence_Store_obj[i].LuxHight[0];
			u16 SrgbID = RD_Sence_Store_obj[i].SrgbID;

				switch(Light_Condition)
				{
					case LESS_THAN_EQUAL:{
						if(Sensor_Lux<=LuxLow){
							if(lux_changle){
								access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
								if(SrgbID)RD_SensorSendRGBScene(SceneID);
								RD_Send_Lux(SceneID);
							}
						}
						break;
					}
					case EQUAL:{
						if(Sensor_Lux==LuxLow){
							if(lux_changle){
							access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
							if(SrgbID)RD_SensorSendRGBScene(SceneID);
							RD_Send_Lux(SceneID);
							}
						}
						break;
					}
					case GREATER_THAN:{
						if(Sensor_Lux>LuxLow){
							if(lux_changle){
							access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
							if(SrgbID)RD_SensorSendRGBScene(SceneID);
							RD_Send_Lux(SceneID);
							}
						}
						break;
					}
					case GRE_LOW_LES_HIGHT:{
						if(Sensor_Lux>LuxLow&&Sensor_Lux<LuxHight){
							if(lux_changle){
							access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
							if(SrgbID)RD_SensorSendRGBScene(SceneID);
							RD_Send_Lux(SceneID);
							}
						}
						break;
					}
					case CLEAR_SCENE_IN_FLASH:{
						RD_ClearSceneInFlash(i);
						break;
					}
				}

			}

		lux_changle=FALSE;
	//}
}


u8 type_device_to_gw_tx_buff[TYPE_DEVICE_BUFF_LEN];

int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	//uart_CSend("Co ban tin");
	//char UART_TempSend[128];
	//sprintf(UART_TempSend,"Messenger type:%d-Content:%x-%x-%x-%x-%x-%x-%x-%x\n",par_len,par[0],par[1],par[2],par[3],par[4],par[5],par[6],par[7]);
	//uart_CSend(UART_TempSend);
	//uart_CSend("..\n");
	uint16_t Message_Header = (par[1]<<8)|par[0];
	if(Message_Header==RD_TYPE_DEVICE_RSP_HEADER){
		for(uint8_t i=0;i<11;i++) type_device_to_gw_tx_buff[i] = RD_NONE;
		type_device_to_gw_tx_buff[0] = (u8)(RD_TYPE_DEVICE_RSP_HEADER);
		type_device_to_gw_tx_buff[1] = (u8)(RD_TYPE_DEVICE_RSP_HEADER>>8);
		type_device_to_gw_tx_buff[2] = RD_SENSOR_TYPE_DEVICE;
		type_device_to_gw_tx_buff[3] = RD_LIGHT_SENSOR_TYPE_DEVICE;
		type_device_to_gw_tx_buff[4] = RD_LIGHT_SENSOR_CB03_LS_BLE_TYPE_DEVICE;
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)type_device_to_gw_tx_buff, TYPE_DEVICE_BUFF_LEN, Gw_addr, 2);
	}
	else if(Message_Header==RD_SAVE_GW_ADDR_HEADER){
		#define GW_RSP_BUFF_LEN     4
		u8 Save_gw_addr_rsp_buff[GW_RSP_BUFF_LEN];
		Gw_addr=cb_par->adr_src;
		RD_FlashWriteGwAddr(Gw_addr);
		Save_gw_addr_rsp_buff[0]=(u8)(RD_SAVE_GW_ADDR_HEADER);
		Save_gw_addr_rsp_buff[1]=(u8)(RD_SAVE_GW_ADDR_HEADER>>8);
		Save_gw_addr_rsp_buff[2]=(u8)(Gw_addr);
		Save_gw_addr_rsp_buff[3]=(u8)(Gw_addr>>8);
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)Save_gw_addr_rsp_buff, GW_RSP_BUFF_LEN, Gw_addr, 2);
	}
	return 0;
}

int RD_Messenger_Process_Scene(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	uint16_t Message_Header;
	Message_Header = (par[1]<<8)|par[0];
	if(Message_Header==RD_HEADER_SENSOR_SAVE_SCENE) RD_FlashSaveSenceData(par);
	if (Message_Header==RD_HEADER_SENSOR_CLEAR_SCENE)
		RD_ClearSceneInFlash(par);
	return 0;
}

int RD_Messenger_Process_Null(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){

	return 0;
}
