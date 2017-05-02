#include "MPU6050.h"


MPU6050_Handle MPU6050_init(void *pMemory,const size_t numBytes) {
	MPU6050_Handle mpu6050Handle;

	  if(numBytes < sizeof(MPU6050_Obj))
	    return((MPU6050_Handle)NULL);

	  // assign the handle
	  mpu6050Handle = (MPU6050_Handle)pMemory;

	  return(mpu6050Handle);
}

void MPU6050_setI2CHandle(MPU6050_Handle handle, I2C_Handle i2cHandle) {
	MPU6050_Obj *obj = (MPU6050_Obj *)handle;
	obj->i2cHandle = i2cHandle;
}

void MPU6050_setI2CAddress(MPU6050_Handle handle, MPU6050_I2C_Addr_e addr) {
	// sets the objects physical I2C address
	MPU6050_Obj *obj = (MPU6050_Obj *)handle;
	obj->i2cAddr = addr;
}


void MPU6050_setup(MPU6050_Handle handle) {
	//set I2C address
	// set I2C handle

	// reset the device
	MPU6050_I2C_WriteByte(handle, MPU6050_RA_PWR_MGMT_1, 0x80);
}

uint16_t MPU6050_I2C_ReadByte(MPU6050_Handle handle, uint16_t reg) {
	MPU6050_Obj *obj = (MPU6050_Obj *)handle;
	return I2C_readData(obj->i2cHandle , (uint16_t)obj->i2cAddr, reg);
}

void MPU6050_I2C_WriteByte(MPU6050_Handle handle, uint16_t reg, uint16_t data) {
	MPU6050_Obj *obj =  (MPU6050_Obj*)handle;
	I2C_writeData(obj->i2cHandle, obj->i2cAddr, reg, data);
}

int16_t MPU6050_getGyro_X(MPU6050_Handle handle) {
	uint16_t high, low;
	uint16_t result = 0;

	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_XOUT_H);
	//MPU6050_I2C_ReadByte(handle, MPU6050_RA_WHO_AM_I); /// !!!!
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_XOUT_L);

	// merge high and low 8 bit readings to produce 16 bit value
	result = (((int16_t)high  << 8)) | (low & 0x00FF);
	return result;

}

int16_t MPU6050_getGyro_Y(MPU6050_Handle handle) {
	int16_t high, low;

	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_YOUT_H);
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_YOUT_L);

	// merge high and low 8 bit readings to produce 16 bit value
	return (((high & (0xFF)) << 8) | (low & 0xFF));

}

int16_t MPU6050_getGyro_Z(MPU6050_Handle handle) {
	int16_t high, low;

	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_ZOUT_H);
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_ZOUT_L);

	// merge high and low 8 bit readings to produce 16 bit value
	return (((high & (0xFF)) << 8) | (low & 0xFF));

}

