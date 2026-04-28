/**
 * @file drv_dm_bmi088.c
 * @author Oyyp
 * @brief description
 * @version 0.1
 * @date 2026-04-23 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_dm_bmi088.h"

dmbmi088imu_t DM_Bmi088imu;

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/**
 * @brief 初始化DMBMI088
 * 
 * @param can_id 
 * @param mst_id 
 * @param hfdcan 
 */
void DM_imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan)
{
	DM_Bmi088imu.can_id=can_id;
	DM_Bmi088imu.mst_id=mst_id;
	DM_Bmi088imu.can_handle=hfdcan;
}

/*
		发送指令
*/
static void imu_send_cmd(uint8_t reg_id,uint8_t ac,uint32_t data)
{
	
	if(DM_Bmi088imu.can_handle==NULL)
		return;
	
	FDCAN_TxHeaderTypeDef tx_header;
	
	uint8_t buf[8]={0xCC,reg_id,ac,0xDD,0,0,0,0};
	memcpy(buf+4,&data,4);
	
	tx_header.DataLength=FDCAN_DLC_BYTES_8;
	tx_header.IdType=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=DM_Bmi088imu.can_id;
	tx_header.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(DM_Bmi088imu.can_handle)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(DM_Bmi088imu.can_handle,&tx_header,buf);
	}
}

void imu_write_reg(uint8_t reg_id,uint32_t data)
{
	imu_send_cmd(reg_id,CMD_WRITE,data);
}

void imu_read_reg(uint8_t reg_id)
{
	imu_send_cmd(reg_id,CMD_READ,0);
}

void imu_reboot()
{
	imu_write_reg(REBOOT_IMU,0);
}

void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI,0);
}

void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI,0);
}

void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM,(uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY,delay);
}

void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE,1);
}

void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE,0);
}

void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD,(uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID,can_id);
}

void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID,mst_id);
}

void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM,0);
}

void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING,0);
}

void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}

void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}

void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}

void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}

void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	DM_Bmi088imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DM_Bmi088imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DM_Bmi088imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	DM_Bmi088imu.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DM_Bmi088imu.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DM_Bmi088imu.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	DM_Bmi088imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	DM_Bmi088imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	DM_Bmi088imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	DM_Bmi088imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	DM_Bmi088imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	DM_Bmi088imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	DM_Bmi088imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(uint8_t* pData)
{
	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	}
}

void DM_BMI088_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    IMU_UpdateData(Buffer);
}

/* Function prototypes -------------------------------------------------------*/

