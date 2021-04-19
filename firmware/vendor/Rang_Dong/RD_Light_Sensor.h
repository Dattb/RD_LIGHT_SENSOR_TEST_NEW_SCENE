/*
 * RD_Light_Sensor.h
 *
 *  Created on: Nov 19, 2020
 *      Author: Dat UTC
 */
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"
#include"Sensor_Flash.h"


#ifndef RD_LIGHT_SENSOR_H_
#define RD_LIGHT_SENSOR_H_		1
#define RD_PIR_SENSOR_H_		2

//#define type_sensor     RD_LIGHT_SENSOR_H_
//type_sensor == RD_LIGHT_SENSOR_H_

// new define 12/11/2020 2:59 PM
#define     TX_BUFF_DATA_LEN                    2
#define     RX_BUFF_DATA_LEN                    2
#define     GW_TX_BUFF_DATA_LEN                 6
#define     SLAVE_DEVICE_ADDR                   0x88
#define     SLAVE_DEVICE_ADDR_LEN               1
#define     I2C_CLK_SPEED                       200000

//volatile unsigned char i2c_tx_buff[TX_BUFF_DATA_LEN] = {0};


#define OPT3001_RESULT_REGISTER                 0x00
#define OPT3001_RESULT_REGISTER_LEN             1
#define OPT3001_CONFIG_REGISTER                 0x01
#define OPT3001_CONFIG_REGISTER_LEN             1
#define RSP_MAX                                 2
#define GATEWAY_ADDR                            0x0001
#define OPT3001_CONFIG_REG_HIGH                 0xC4   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_HIGH                 0xC4   // xem trong datasheet phan configure register
#define OPT3001_CONFIG_REG_LOW                  0x10   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_LOW                  0x14   // xem trong datasheet phan configure register


#define SENSOR_POWER_TYPE                      0x0001
#define REMOTE_DC_MODULE_TYPE                  0x0002
#define REMOTE_AC_MODULE_TYPE                  0x0003
#define LIGHT_SENSOR_MODULE_TYPE               0x0004
#define PIR_SENSOR_MODULE_TYPE                 0x0005








#define TYPE_DEVICE_BUFF_LEN       			11
#define RD_SAVE_GW_ADDR_HEADER        		0x0002
#define RD_TYPE_DEVICE_RSP_HEADER        	0x0003


#define RD_SENSOR_TYPE_DEVICE 							3
#define RD_LIGHT_SENSOR_TYPE_DEVICE 					1
#define RD_LIGHT_SENSOR_CB03_LS_BLE_TYPE_DEVICE 		1
#define RD_NONE   0

#define RD_OPCODE_TYPE_DEVICE_SEND			(0xE0)
#define RD_OPCODE_TYPE_DEVICE_RSP			(0xE1)
#define RD_OPCODE_SCENE_SEND		(0xE2)
#define RD_OPCODE_SCENE_RSP			(0xE3)



#define RD_HEADER_SENSOR_SEND_RGB_SCENE      0x0200
#define RD_HEADER_SENSOR_SAVE_SCENE          0x0201
#define RD_HEADER_SENSOR_CLEAR_SCENE         0x0202

#define RD_VENDOR_EN	1








void RD_light_sensor_tx (u16 loop);
void RD_Led_init(void);
unsigned int RD_power_read();
void RD_ADC_init (u32 gpio);
void  Led_toggle(void);
unsigned int CalculateLux(unsigned int rsp_lux);
void delay_init(u16 time);
void time_poll_change (u8*par,mesh_cb_fun_par_t *cb_par);
void RD_Send_LightSensorPower();
void RD_Send_Lux(u16 SceneID);
void RD_SensorSendRGBScene(u16 app_rgbid);
void RD_LightSensorControlSence(u16 Sensor_Lux);
int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int RD_Messenger_Process_Scene(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int RD_Messenger_Process_Null(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
#endif /* RD_LIGHT_SENSOR_H_ */



