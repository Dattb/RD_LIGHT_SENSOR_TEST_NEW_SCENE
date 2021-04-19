/*
 * Sensor_Flash.c
 *
 *  Created on: Jan 28, 2021
 *      Author: Dat UTC
 */



#include"Sensor_Flash.h"
#include "RD_Light_Sensor.h"
RD_SensorSencesStoreTypedef RD_Sence_Store_obj[NUMBER_OF_SCENE];
u8 SceneID_Store[NUMBER_OF_SCENE];
u8  RD_Sence_Flash_Read_Buff [FLASH_BUFF_LEN];
u8  RD_Reset_Flash_Read_Buff [FLASH_BUFF_LEN];

u8  RD_Sence_Flash_Write_Buff[FLASH_BUFF_LEN];
extern u16 Gw_addr;

void RD_FlashSaveSenceData(u8 *RD_ScenePar)
{
   // u8 Sence_Number = RD_ScenePar[2];
/***************************************************************
 * Mau data luu scene tren cam bien
 */
/*
 * 	RD_Sence_Store_obj[Scene_Number].Header[0] 				= RD_SencePar[0];
 * 	RD_Sence_Store_obj[Scene_Number].Header[1] 				= RD_SencePar[1];
	RD_Sence_Store_obj[Scene_Number].SenceID[0]   			= RD_SencePar[2];
	RD_Sence_Store_obj[Scene_Number].SenceID[1]   			= RD_SencePar[3];
	RD_Sence_Store_obj[Scene_Number].Light_Condition 		= RD_SencePar[4];
	RD_Sence_Store_obj[Scene_Number].PIR_Condition 			= RD_SencePar[5];
	RD_Sence_Store_obj[Scene_Number].LuxLow[0]     			= RD_SencePar[6];
	RD_Sence_Store_obj[Scene_Number].LuxLow[1]     			= RD_SencePar[7];
	RD_Sence_Store_obj[Scene_Number].LuxHight[0]   			= RD_SencePar[8];
	RD_Sence_Store_obj[Scene_Number].LuxHight[1]  			= RD_SencePar[9];
	RD_Sence_Store_obj[Scene_Number].SrgbID  				= RD_SencePar[10];
 */

    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer,New_Scene;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *) RD_ScenePar;
    New_Scene =*RD_Sence_Store_pointer;
    u8 Check_Scene_loop=0;
    for(u8 i=0;i<NUMBER_OF_SCENE;i++){
    	//if(RD_Sence_Store_obj[i].SceneID==0xff){

    		if(RD_Sence_Store_obj[i].SceneID[0] == New_Scene.SceneID[0]&&RD_Sence_Store_obj[i].SceneID[1] == New_Scene.SceneID[1])
    			Check_Scene_loop=1;

//    		else
//    			{
//        		RD_ScenePar[0]=0xee;
//        		RD_ScenePar[1]=0xff;
//    				mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, GATEWAY_ADDR, RSP_MAX);
//    			}
    	//}

    }

    if(Check_Scene_loop==0){
        for(u8 i=0;i<NUMBER_OF_SCENE;i++){
              u16 EmptySceneID = (RD_Sence_Store_obj[i].SceneID[1]<<8)|RD_Sence_Store_obj[i].SceneID[0];
        	if(EmptySceneID==0xffff||EmptySceneID==0x0000){
        		RD_Sence_Store_obj[i] = *RD_Sence_Store_pointer;
				#define RSP_MAX              2
				mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, GATEWAY_ADDR, RSP_MAX);
        		break;
        	}
        }
        Check_Scene_loop=0;
    }

    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *)RD_Sence_Flash_Write_Buff;
    for(u8 i=0;i<NUMBER_OF_SCENE;i++) *(RD_Sence_Store_pointer+i) = RD_Sence_Store_obj[i];

	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Write_Buff);
	RD_FlashReadSceneData ();



}



void RD_FlashWriteGwAddr(uint16_t addr){

	RD_Sence_Flash_Write_Buff[NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ]=(u8)(addr);
	RD_Sence_Flash_Write_Buff[NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ+1]=(u8)(addr>>8);

	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Write_Buff);
	RD_FlashReadSceneData ();
}






extern u8 reset_cnt;
void RD_FlashReadSceneData ()
{
	//flash_read_page (FACTORY_RESET_FLASH_ADDR, FLASH_BUFF_LEN, RD_Reset_Flash_Read_Buff);
	flash_read_page (SCENE_FLASH_ADDR, FLASH_BUFF_LEN, RD_Sence_Flash_Read_Buff);

//	reset_cnt = RD_Reset_Flash_Read_Buff[0];

    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *)RD_Sence_Flash_Read_Buff;
	for(u8 i=0;i<NUMBER_OF_SCENE;i++) RD_Sence_Store_obj[i] = *(RD_Sence_Store_pointer + i);

	uint16_t Flash_GwAddr;
	Flash_GwAddr = 	(RD_Sence_Flash_Read_Buff[NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ+1]<<8)|
					(RD_Sence_Flash_Read_Buff[NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ]);
	if(Flash_GwAddr!=0xff && Flash_GwAddr!=0x00) Gw_addr = Flash_GwAddr;
	else Gw_addr=0x0001;
}

void RD_ClearAllSceneInFlash()
{
	//u8 RD_Sence_Flash_Clear_Buff[FLASH_BUFF_LEN] = {0};
	flash_erase_sector(SCENE_FLASH_ADDR);
	//flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Clear_Buff);
	//RD_FlashWriteGwAddr(Gw_addr);
	RD_FlashReadSceneData ();
}

void RD_ClearSceneInFlash(u8 *RD_ScenePar)
{
    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer,Scene_obj;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *) RD_ScenePar;
    Scene_obj=*RD_Sence_Store_pointer;

	RD_SensorSencesStoreTypedef RD_Clear_scene = {0};
	for(u8 i=0;i<NUMBER_OF_SCENE;i++){
		if(RD_Sence_Store_obj[i].SceneID[0]==Scene_obj.SceneID[0]&&RD_Sence_Store_obj[i].SceneID[1]==Scene_obj.SceneID[1]){
		    RD_Sence_Store_obj[i]= RD_Clear_scene;
			#define RSP_MAX              2
			mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, GATEWAY_ADDR, RSP_MAX);
		}
	}
    RD_SensorSencesStoreTypedef *RD_Sence_Write_pointer;
    RD_Sence_Write_pointer = (RD_SensorSencesStoreTypedef *) RD_Sence_Flash_Write_Buff;
    for(u8 i=0;i<NUMBER_OF_SCENE;i++) *(RD_Sence_Write_pointer+i) = RD_Sence_Store_obj[i];

	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Write_Buff);
	RD_FlashReadSceneData ();

}



extern u8 reset_cnt;
void RD_Write_reset_cnt(u8 data)
{
	RD_Factory_Reset_Flash_Write_Buff[0]=data;
	flash_erase_sector(0x23000);
	flash_write_page(0x23000,1,RD_Factory_Reset_Flash_Write_Buff);
	RD_FlashReadSceneData ();
}




