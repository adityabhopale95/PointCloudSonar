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

void initialize_i2c(int fd, int address){
  int successful = ioctl(fd, I2C_SLAVE, address);
  printf("Success: %d\n", successful);
  if(fd < 0){
    printf("Error: IMU I2C Failed\n");
  }
}

void write_i2c(int fd, uint8_t reg_add, uint8_t value){
  uint8_t data[]={reg_add,value};
  write(fd, data, ARRAY_SIZE(data));
}

int main(int argc, char *argv[]){
  int i2c_file;
  int adapter_nr = 0;
  char filen[20];
  snprintf(filen,19,"/dev/i2c-2", adapter_nr);

  int address;
  uint8_t status;
  int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

  i2c_file = open(filen, O_RDWR);
  if(i2c_file < 0){
    perror("Unable to open I2C control file\n");
    exit(1);
  }

  printf("i2c file: %d\n", i2c_file);
  address = DEV_ADD;

  initialize_i2c(i2c_file,address);

  int8_t power_data = i2c_smbus_read_byte_data(i2c_file, MPU_POWER1);
  printf("Power data: %d\n", int(power_data));
  i2c_smbus_write_byte_data(i2c_file, MPU_POWER1, ~(1 << 6) & power_data);

  accel_x = i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_L);
  accel_y = i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_L);
  accel_z = i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_L);

  gyro_x = i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_L);
  gyro_y = i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_L);
  gyro_z = i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_L);
  
  set_mag_registers(i2c_file);

  mag_x = i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_L);
  mag_y = i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_L);
  mag_z = i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_H) << 8 |
            i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_L);

//  while(1){
/*    initialize_i2c(i2c_file,address);

    accel_x = i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_L);
    accel_y = i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_L);
    accel_z = i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_L);

    gyro_x = i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_L);
    gyro_y = i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_L);
    gyro_z = i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_L);

    set_mag_registers(i2c_file);

    mag_x = i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, X_MAG_ADD_L);
    mag_y = i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Y_MAG_ADD_L);
    mag_z = i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_H) << 8 |
              i2c_smbus_read_byte_data(i2c_file, Z_MAG_ADD_L);
*/
    printf("accel_x: %d accel_y: %d accel_z: %d\n", int(accel_x), int(accel_y), int(accel_z));
    printf("gyro_x: %d gyro_y: %d gyro_z: %d\n", int(gyro_x), int(gyro_y), int(gyro_z));
    printf("mag_x: %d mag_y: %d mag_z: %d\n\n", int(mag_x), int(mag_y), int(mag_z));
//  }
}
