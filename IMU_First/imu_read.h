#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <math.h>
#include <stdint.h>

#define DEV_ADD 0x68

#define X_ACC_ADD_H 0x3b
#define Y_ACC_ADD_H 0x3d
#define Z_ACC_ADD_H 0x3f
#define X_ACC_ADD_L 0x3c
#define Y_ACC_ADD_L 0x3e
#define Z_ACC_ADD_L 0x40

#define X_GYRO_ADD_H 0x43
#define Y_GYRO_ADD_H 0x45
#define Z_GYRO_ADD_H 0x47
#define X_GYRO_ADD_L 0x44
#define Y_GYRO_ADD_L 0x46
#define Z_GYRO_ADD_L 0x48

#define X_MAG_ADD_H 0x04
#define Y_MAG_ADD_H 0x06
#define Z_MAG_ADD_H 0x08
#define X_MAG_ADD_L 0x03
#define Y_MAG_ADD_L 0x05
#define Z_MAG_ADD_L 0x07

#define MPU_POWER1 0x6b
#define MPU_POWER2 0x6c

#define INT_PIN_CONF 0x37

int MAG_CNTL1 = 0x0a;
int MAG_CNTL2 = 0x0b;
int MAG_ASTC = 0x0c;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

void initialize_i2c(int fd,int address);

void write_i2c(int fd, uint8_t reg_add, uint8_t value);

void set_mag_registers(int fd){
  int adapter_nr = 0;
  char filen[20];
  snprintf(filen,19,"/dev/i2c-2", adapter_nr);

  write_i2c(fd, 0x37, 0x22);

  fd = open(filen, O_RDWR);
  if(fd < 0){
    perror("Unable to open I2C control file\n");
    exit(1);
  }
  initialize_i2c(fd, MAG_ASTC);
  sleep(0.1);
  write_i2c(fd, MAG_CNTL2, 0x01);
  write_i2c(fd, MAG_CNTL2, 0x00);
  write_i2c(fd, MAG_CNTL1, 0x1f);
  write_i2c(fd, MAG_CNTL1, 0x10);
  write_i2c(fd, MAG_CNTL1, 0x12);
  sleep(0.1);
}
