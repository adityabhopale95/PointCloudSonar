#include "imu_read.h"
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <math.h>
#include <stdint.h>

void initialize_i2c(int address){
  int adapter_nr = 0;
  char filen[20];
  snprintf(filen,19,"/dev/i2c-2", adapter_nr);
  close(i2c_file);
  i2c_file = open(filen, O_RDWR);
  if(i2c_file < 0){
    perror("Unable to open I2C control file\n");
    exit(1);
  }

  printf("i2c file: %d\n", i2c_file);

  int successful = ioctl(i2c_file, I2C_SLAVE, address);
  printf("Success: %d\n", successful);
  if(i2c_file < 0){
    printf("Error: IMU I2C Failed\n");
  }
}

void write_i2c(uint8_t reg_add, uint8_t value){
  uint8_t data[]={reg_add,value};
  write(i2c_file, data, ARRAY_SIZE(data));
}

int main(int argc, char *argv[]){
  int address;
  uint8_t status;
  int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
  int16_t mag_count[3];
  address = DEV_ADD;

  initialize_mpu();

  initialize_mag(mag_Calib);

  while(1){
    initialize_i2c(DEV_ADD);
    status = i2c_smbus_read_byte_data(i2c_file, INT_STATUS);
    printf("Status: %d\n", status);
    if(status & 0x01){
      accel_x = (i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_L)) * Ares;
      accel_y = (i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_L)) * Ares;
      accel_z = (i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_L)) * Ares;

      gyro_x = (i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_L)) * Gres;
      gyro_y = (i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_L)) * Gres;
      gyro_z = (i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_H) << 8 |
                i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_L)) * Gres;

      initialize_i2c(MAG_ASTC);
      read_data_mag(mag_count);
      mag_bias[0] =+ 470.0;
      mag_bias[1] =+ 120.0;
      mag_bias[2] =+ 125.0;

      mag_x = (float(mag_count[0]) * Mres * mag_Calib[0]) - mag_bias[0];
      mag_y = (float(mag_count[1]) * Mres * mag_Calib[1]) - mag_bias[1];
      mag_z = (float(mag_count[2]) * Mres * mag_Calib[2]) - mag_bias[2];
      printf("accel_x: %d accel_y: %d accel_z: %d\n", int(accel_x), int(accel_y), int(accel_z));
      printf("gyro_x: %d gyro_y: %d gyro_z: %d\n", int(gyro_x), int(gyro_y), int(gyro_z));
      printf("mag_x: %d mag_y: %d mag_z: %d\n\n", int(mag_x), int(mag_y), int(mag_z));
    }
  }
}

void initialize_mpu(){
  initialize_i2c(DEV_ADD);

  write_i2c(MPU_POWER1, 0x00);
  sleep(0.1);
  write_i2c(MPU_POWER1, 0x01);
  sleep(0.2);

  write_i2c(CONFIG_MPU, 0x03);
  write_i2c(CONFIG_MPU, 0x04);

  uint8_t c = i2c_smbus_read_byte_data(i2c_file, GYRO_CONFIG);
  c = c & ~0x03;
  c = c & ~0x18;
  write_i2c(GYRO_CONFIG, c);

  uint8_t d = i2c_smbus_read_byte_data(i2c_file, ACCEL_CONFIG);
  d = d & ~0x18;
  write_i2c(ACCEL_CONFIG, d);

  uint8_t e = i2c_smbus_read_byte_data(i2c_file, ACCEL_CONFIG2);
  e = e & ~0x0F;
  e = e | 0x03;
  write_i2c(ACCEL_CONFIG2, e);


  write_i2c(INT_PIN_CONF, 0x22);
  write_i2c(INT_ENABLE, 0x01);
  sleep(0.1);
}

void initialize_mag(float * destination){
  uint8_t rawData[3];
  initialize_i2c(MAG_ASTC);
  write_i2c(MAG_CNTL1, 0x00);
  sleep(0.01);
  write_i2c(MAG_CNTL2, 0x01);
  sleep(0.01);
  rawData[0] = i2c_smbus_read_byte_data(i2c_file, MAG_SENSE_X);
  rawData[1] = i2c_smbus_read_byte_data(i2c_file, MAG_SENSE_Y);
  rawData[2] = i2c_smbus_read_byte_data(i2c_file, MAG_SENSE_Z);

  destination[0] = (float(rawData[0]-128) / 256.0) + 1.0;
  destination[1] = (float(rawData[1]-128) / 256.0) + 1.0;
  destination[2] = (float(rawData[2]-128) / 256.0) + 1.0;

  write_i2c(MAG_CNTL1, 0x00);
  sleep(0.01);

  write_i2c(MAG_CNTL1, 0x12);
  sleep(0.01);
}

void read_data_mag(int16_t *destination){
  uint8_t rawData[6];
  uint8_t status1;
  uint8_t status2;
  status1 = i2c_smbus_read_byte_data(i2c_file, MAG_STATUS1);
  if(status1 & 0x01){
    rawData[0] = i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_H);
    rawData[1] = i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_L);
    rawData[2] = i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_H);
    rawData[3] = i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_L);
    rawData[4] = i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_H);
    rawData[5] = i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_L);

    status2 = i2c_smbus_read_byte_data(i2c_file, MAG_STATUS2);
    if(!(status2 & 0x08)){
      destination[0] = (int16_t(rawData[1]) << 8) | rawData[0];
      destination[1] = (int16_t(rawData[3]) << 8) | rawData[2];
      destination[2] = (int16_t(rawData[5]) << 8) | rawData[4];
    }
  }
}
