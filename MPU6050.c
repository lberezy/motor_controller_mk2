#include "MPU6050.h"

#include <stdio.h>
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
	//return I2C_readData(obj->i2cHandle , (uint16_t)obj->i2cAddr, reg);

	uint16_t write_len = 1;
	uint16_t write_data[1] = {reg};
	uint16_t read_len = 1;
	uint16_t read_data[1];
	I2C_readBytes(obj->i2cHandle, (uint16_t)obj->i2cAddr, write_data, write_len, read_data, read_len);
	return read_data[0];

}

void MPU6050_I2C_WriteByte(MPU6050_Handle handle, uint16_t reg, uint16_t data) {
	MPU6050_Obj *obj =  (MPU6050_Obj*)handle;
	I2C_writeData(obj->i2cHandle, obj->i2cAddr, reg, data);
}

int16_t MPU6050_getGyro_X(MPU6050_Handle handle) {
	uint16_t high = 0, low = 0;
	uint16_t whoami = 0;
	int16_t result = 0;

	whoami = MPU6050_I2C_ReadByte(handle, MPU6050_RA_WHO_AM_I); /// !!!!
	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_XOUT_H);
	//whoami = MPU6050_I2C_ReadByte(handle, MPU6050_RA_WHO_AM_I); /// !!!!
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_XOUT_L);

	/* --- PRINTF_BYTE_TO_BINARY macros --- */
	// print MSBit first
	#define _PRINTF_BYTE_TO_BINARY(i, shift)    \
	    (((i) & (0x80 << (shift))) ? '1' : '0'), \
	    (((i) & (0x40 << (shift))) ? '1' : '0'), \
	    (((i) & (0x20 << (shift))) ? '1' : '0'), \
	    (((i) & (0x10 << (shift))) ? '1' : '0'), \
	    (((i) & (0x08 << (shift))) ? '1' : '0'), \
	    (((i) & (0x04 << (shift))) ? '1' : '0'), \
	    (((i) & (0x02 << (shift))) ? '1' : '0'), \
	    (((i) & (0x01 << (shift))) ? '1' : '0')

	#define PRINTF_BINARY_PATTERN_INT8 \
	    "%c%c%c%c%c%c%c%c"
	#define PRINTF_BYTE_TO_BINARY_INT8(i)  \
	    _PRINTF_BYTE_TO_BINARY(i, 0)

	#define PRINTF_BINARY_PATTERN_INT16 \
	    PRINTF_BINARY_PATTERN_INT8 \
	    PRINTF_BINARY_PATTERN_INT8
	#define PRINTF_BYTE_TO_BINARY_INT16(i)  \
	    _PRINTF_BYTE_TO_BINARY(i, 0), \
	    _PRINTF_BYTE_TO_BINARY(i, 8)

	#define PRINTF_BINARY_PATTERN_INT32 \
	    PRINTF_BINARY_PATTERN_INT16 \
	    PRINTF_BINARY_PATTERN_INT16
	#define PRINTF_BYTE_TO_BINARY_INT32(i)  \
	    _PRINTF_BYTE_TO_BINARY(i,  0), \
	    _PRINTF_BYTE_TO_BINARY(i,  8), \
	    _PRINTF_BYTE_TO_BINARY(i, 16), \
	    _PRINTF_BYTE_TO_BINARY(i, 24)

	//printf("H: " PRINTF_BINARY_PATTERN_INT8 "L: " PRINTF_BINARY_PATTERN_INT8 "\r\n" , PRINTF_BYTE_TO_BINARY_INT8(high), PRINTF_BYTE_TO_BINARY_INT8(low));
	//printf("H: %#x L: %#x \r\n", high, low);`
	//printf("WHOAMI: %#x \r\n", whoami);
	// merge high and low 8 bit readings to produce 16 bit value
	//result = (((int16_t)high  << 8)) | (low & 0x00FF);

	result = high;
	result <<= 8;
	result += low;
	return result;
	// return two's complement
	//if (result >= 0x8000) {
	//	return -((65535 - result) + 1);
	//} else {
		//return result;
	//}

	//long t = ((long)high << 8) | low;
	//if (t >= 32768) {
	//	t -= 65536;
	//}
	//printf("%d\r\n", t);
	//return t;

}

int16_t MPU6050_getGyro_Y(MPU6050_Handle handle) {
	uint16_t high, low;

	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_YOUT_H);
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_YOUT_L);

	// merge high and low 8 bit readings to produce 16 bit value
	return (((high & (0xFF)) << 8) | (low & 0xFF));

}

int16_t MPU6050_getGyro_Z(MPU6050_Handle handle) {
	uint16_t high, low;

	high = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_ZOUT_H);
	low = MPU6050_I2C_ReadByte(handle, MPU6050_RA_GYRO_ZOUT_L);

	// merge high and low 8 bit readings to produce 16 bit value
	return (((high & (0xFF)) << 8) | (low & 0xFF));

}

void MPU6050_getGyro_All(MPU6050_Handle handle, int16_t* data) {
	MPU6050_Obj *obj =  (MPU6050_Obj*)handle;

	uint16_t write_len = 1;
	uint16_t write_data[1] = {MPU6050_RA_GYRO_XOUT_H}; // start adress
	uint16_t read_len = 6;
	uint16_t read_data[6];
	I2C_readBytes(obj->i2cHandle, (uint16_t)obj->i2cAddr, write_data, write_len, read_data, read_len);

	data[0] = data[0] << 8;
	data[0] += data[1]; // x axis gyro
	data[1] = ((data[2] << 8) | data[3]); // y axis gyro
	data[2] = ((data[4] << 8) | data[5]); // z axis gyro

}

