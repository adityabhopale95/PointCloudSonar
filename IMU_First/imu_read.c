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
#include <string.h>

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

//  printf("i2c file: %d\n", i2c_file);

  int successful = ioctl(i2c_file, I2C_SLAVE, address);
//  printf("Success: %d\n", successful);
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
  float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
  int16_t mag_count[3];
  int16_t acc_count[3];
  int16_t gyro_count[3];
  address = DEV_ADD;
  FILE *fp_wr;
  char *fp_name = "a";
  float x_out,y_out,z_out;
  //fp_name = strcat(fp_name,".csv");
  fp_wr = fopen(fp_name,"w+");
  initialize_mpu();

  initialize_mag(mag_Calib);

  while(1){
    initialize_i2c(DEV_ADD);
    status = i2c_smbus_read_byte_data(i2c_file, INT_STATUS);
//    printf("Status: %d\n", status);
    if(status & 0x01){
      read_data_acc(acc_count);
      accel_x = (float)(acc_count[0]) * Ares;
      accel_y = (float)(acc_count[1]) * Ares;
      accel_z = (float)(acc_count[2]) * Ares;

      read_data_gyro(gyro_count);
      gyro_x = (float)(gyro_count[0]) * Gres;
      gyro_y = (float)(gyro_count[1]) * Gres;
      gyro_z = (float)(gyro_count[2]) * Gres;

      initialize_i2c(MAG_ASTC);
      read_data_mag(mag_count);
      mag_bias[0] =+ 470.0;
      mag_bias[1] =+ 120.0;
      mag_bias[2] =+ 125.0;

      mag_x = (float(mag_count[0]) * Mres * mag_Calib[0]) - mag_bias[0];
      mag_y = (float(mag_count[1]) * Mres * mag_Calib[1]) - mag_bias[1];
      mag_z = (float(mag_count[2]) * Mres * mag_Calib[2]) - mag_bias[2];
    }

    MadgwickAHRSupdateIMUOpt((gyro_x*(PI/180)), (gyro_y*(PI/180)), (gyro_z*(PI/180)), accel_x, accel_y, accel_z);
    //toEulerianAngle(q0, q1, q2, q3);
    toEulerAngle(q0, q1, q2, q3);

    if(yaw < 0.0){
      yaw = yaw + (2*PI);
    }
    x_out = r * sinf(pitch) * cosf(yaw);
    y_out = r * sinf(pitch) * sinf(yaw);
    z_out = r * cosf(yaw);

    fprintf(fp_wr, "%f,%f,%f\n",x_out, y_out, z_out);
    printf("accel_x: %f accel_y: %f accel_z: %f\n", accel_x, accel_y, accel_z);
    printf("gyro_x: %f gyro_y: %f gyro_z: %f\n", gyro_x, gyro_y, gyro_z);
    printf("mag_x: %f mag_y: %f mag_z: %f\n", mag_x, mag_y, mag_z);
    printf("q0: %f, q1: %f,q2: %f,q3: %f\n", q0,q1,q2,q3);
    printf("roll: %f pitch: %f yaw: %f\n\n", (roll*(180.0/PI)), (pitch*(180.0/PI)), (yaw*(180.0/PI)));
    sleep(0.2);
  }
}

void initialize_mpu(){
  initialize_i2c(DEV_ADD);

  write_i2c(MPU_POWER1, 0x00);
  sleep(0.1);
  write_i2c(MPU_POWER1, 0x01);
  sleep(0.2);

  write_i2c(CONFIG_MPU, 0x03);
  write_i2c(DIV_SMPLR, 0x04);

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

  destination[0] = (((float)rawData[0]-128.0) / 256.0) + 1.0;
  destination[1] = (((float)rawData[1]-128.0) / 256.0) + 1.0;
  destination[2] = (((float)rawData[2]-128.0) / 256.0) + 1.0;

  write_i2c(MAG_CNTL1, 0x00);
  sleep(0.01);

  write_i2c(MAG_CNTL1, 0x12);
  sleep(0.01);
}

void read_data_acc(int16_t *destination){
  uint8_t rawData[6];
  rawData[0] = i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_H);
  rawData[1] = i2c_smbus_read_byte_data(i2c_file, X_ACC_ADD_L);
  rawData[2] = i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_H);
  rawData[3] = i2c_smbus_read_byte_data(i2c_file, Y_ACC_ADD_L);
  rawData[4] = i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_H);
  rawData[5] = i2c_smbus_read_byte_data(i2c_file, Z_ACC_ADD_L);

  destination[0] = (int16_t)rawData[0] << 8 | rawData[1];
  destination[1] = (int16_t)rawData[2] << 8 | rawData[3];
  destination[2] = (int16_t)rawData[4] << 8 | rawData[5];
}

void read_data_gyro(int16_t *destination){
  uint8_t rawData[6];
  rawData[0] = i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_H);
  rawData[1] = i2c_smbus_read_byte_data(i2c_file, X_GYRO_ADD_L);
  rawData[2] = i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_H);
  rawData[3] = i2c_smbus_read_byte_data(i2c_file, Y_GYRO_ADD_L);
  rawData[4] = i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_H);
  rawData[5] = i2c_smbus_read_byte_data(i2c_file, Z_GYRO_ADD_L);

  destination[0] = (int16_t)rawData[0] << 8 | rawData[1];
  destination[1] = (int16_t)rawData[2] << 8 | rawData[3];
  destination[2] = (int16_t)rawData[4] << 8 | rawData[5];
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
      destination[0] = (int16_t)(rawData[1]) << 8 | rawData[0];
      destination[1] = (int16_t)(rawData[3]) << 8 | rawData[2];
      destination[2] = (int16_t)(rawData[5]) << 8 | rawData[4];
    }
  }
}

void MadgwickAHRSupdateOpt(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float calc_mul1, calc_mul2, calc_mul3, calc_mul4, calc_mul5, calc_mul6, calc_mul7, calc_mul8, calc_mul9, calc_mul10;
	float reci_sample = (1.0f / sampleFreq);

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMUOpt(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrtOpt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrtOpt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		calc_mul1 = (2.0f * q1q3 - _2q0q2 - ax);
		calc_mul2 =	(2.0f * q0q1 + _2q2q3 - ay);
		calc_mul3 = (0.5f - q2q2 - q3q3);
		calc_mul4 = (q1q2 - q0q3);
		calc_mul5 = (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
		calc_mul6 = (q0q1 + q2q3);
		calc_mul7 = (q1q3 - q0q2);
		calc_mul8 = (q0q2 + q1q3);
		calc_mul9 = (0.5f - q1q1 - q2q2);

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * calc_mul1 + _2q1 * calc_mul2 - _2bz * q2 * (_2bx * calc_mul3 + _2bz * calc_mul7 - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * calc_mul4 + _2bz * calc_mul6 - my) + _2bx * q2 * (_2bx * calc_mul8 + _2bz * calc_mul9 - mz);
		s1 = _2q3 * calc_mul1 + _2q0 * calc_mul2 - 4.0f * q1 * calc_mul5 + _2bz * q3 * (_2bx * calc_mul3 + _2bz * calc_mul7 - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * calc_mul4 + _2bz * calc_mul6 - my) + (_2bx * q3 - _4bz * q1) * (_2bx * calc_mul8 + _2bz * calc_mul9 - mz);
		s2 = -_2q0 * calc_mul1 + _2q3 * calc_mul2 - 4.0f * q2 * calc_mul5 + (-_4bx * q2 - _2bz * q0) * (_2bx * calc_mul3 + _2bz * calc_mul7 - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * calc_mul4 + _2bz * calc_mul6 - my) + (_2bx * q0 - _4bz * q2) * (_2bx * calc_mul8 + _2bz * calc_mul9 - mz);
		s3 = _2q1 * calc_mul1 + _2q2 * calc_mul2 + (-_4bx * q3 + _2bz * q1) * (_2bx * calc_mul3 + _2bz * calc_mul7 - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * calc_mul4 + _2bz * calc_mul6 - my) + _2bx * q1 * (_2bx * calc_mul8 + _2bz * calc_mul9 - mz);
		recipNorm = invSqrtOpt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * reci_sample;
	q1 += qDot2 * reci_sample;
	q2 += qDot3 * reci_sample;
	q3 += qDot4 * reci_sample;

	// Normalise quaternion
	recipNorm = invSqrtOpt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void MadgwickAHRSupdateIMUOpt(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float reci_sample = (1.0f / sampleFreq);

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrtOpt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrtOpt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * reci_sample;
	q1 += qDot2 * reci_sample;
	q2 += qDot3 * reci_sample;
	q3 += qDot4 * reci_sample;

	// Normalise quaternion
	recipNorm = invSqrtOpt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float invSqrtOpt(float x) {
	union {
		float f;
		long i;
	} conv;

	float a;
	a = x * 0.5F;
	const float threehalfs = 1.5f;
	conv.f = x;
	conv.i =  0x5f3759df - ( conv.i >> 1 );
	conv.f = conv.f * (threehalfs - ( a * conv.f * conv.f ));
	return conv.f;
}

void toEulerAngle(float q0, float q1, float q2, float q3 )
{
	// roll (x-axis rotation)
	float sinr = 2.0 * (q0 * q1 + q2 * q3);
	float cosr = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
	roll = atan2f(sinr, cosr);

	// pitch (y-axis rotation)
	float sinp = 2.0 * (q0 * q2 - q3 * q1);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asinf(sinp);

	// yaw (z-axis rotation)
	float siny = 2.0 * (q0 * q3 + q1 * q2);
	float cosy = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
	yaw = atan2f(siny, cosy);
}

void toEulerianAngle(float q0,float q1,float q2, float q3)
{
	float ysqr = q2 * q2;
	float t0 = -2.0f * (ysqr + q3 * q3) + 1.0f;
	float t1 = +2.0f * (q1 * q2 - q0 * q3);
	float t2 = -2.0f * (q1 * q3 + q0 * q2);
	float t3 = +2.0f * (q2 * q3 - q0 * q1);
	float t4 = -2.0f * (q1 * q1 + ysqr) + 1.0f;

	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;

	pitch1 = asinf(t2);
	roll1 = atan2f(t3, t4);
	yaw1 = atan2f(t1, t0);
}
