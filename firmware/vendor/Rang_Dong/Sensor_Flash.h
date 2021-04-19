/*
 * Sensor_Flash.h
 *
 *  Created on: Jan 28, 2021
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

#ifndef SENSOR_FLASH_H_
#define SENSOR_FLASH_H_




#define SCENE_FLASH_ADDR                             0x78000//0x022000
#define FACTORY_RESET_FLASH_ADDR                             0x23000//0x022000
#define NUMBER_OF_SCENE                        20
#define BYTE_OF_ONE_SCENE_OBJ				   11
#define GW_ADDR_LEN							   2
#define FLASH_BUFF_LEN                         (NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ+GW_ADDR_LEN)






#define RD_FACTORY_RESET_CNT_LEN  1
unsigned char RD_Factory_Reset_Flash_Write_Buff[RD_FACTORY_RESET_CNT_LEN];




enum{
	NOT_USE                     = 0,

	LESS_THAN_EQUAL				= 1,
	MOTION						= 1,

	EQUAL						= 2,
	NOT_MOTION					= 2,

	GREATER_THAN				= 3,

	GRE_LOW_LES_HIGHT 			= 4,

	CLEAR_SCENE_IN_FLASH 		= 5,
	CLEAR_ALL_SCENE_IN_FLASH 	= 5,
};


typedef struct{
	u8 Header[2];
	u8 SceneID [2];
	u8 Light_Condition;
	u8 PIR_Condition;
	u8 LuxLow[2];
	u8 LuxHight[2];
	u8 SrgbID;
}RD_SensorSencesStoreTypedef;



void RD_FlashSaveSenceData(u8 *RD_SencePar);
void RD_FlashReadSceneData ();
void RD_ClearAllSceneInFlash();
void RD_ClearSceneInFlash(u8 *RD_ScenePar);
void RD_Write_reset_cnt(u8 data);
void RD_FlashWriteGwAddr(uint16_t addr);
#endif /* SENSOR_FLASH_H_ */
