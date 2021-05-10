long acc_x, acc_y, acc_z; //Raw data
int gyro_x, gyro_y, gyro_z; //Raw data
double prev_side;

void init_imu() {
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Configure Imu
}

void update_imu_quat() {
  double converted_gyro_x, converted_gyro_y, converted_gyro_z;         //Converted to °/s

  double angle_gyro_x, angle_gyro_y, angle_gyro_z;
  double total_acc;                                                    //Total force vector
  read_mpu_6050_data();                                                //Pull data from imu

  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro value
  
  converted_acc_x = acc_x / (double)ACCRATE;                           //Convert from raw value to m/s²
  converted_acc_y = acc_y / (double)ACCRATE;
  converted_acc_z = acc_z / (double)ACCRATE;

  converted_gyro_x = gyro_x / GYRORATE;                                //Convert from raw value to °/s
  converted_gyro_y = gyro_y / GYRORATE;
  converted_gyro_z = gyro_z / GYRORATE;
  Quaternion acc = Quaternion(converted_acc_x, converted_acc_y, converted_acc_z);
  acc.normalize();
  float acc_angle = acc.rotation_between_vectors(Quaternion(1, 0, 0)).dot_product(Quaternion());
  angle_y = (acc.d > 0)?-acos(acc_angle)*2:acos(acc_angle)*2;
  corrected_angle = angle_y;

  //if(converted_acc_z < 0.075) corrected_angle *= -1;
}

void update_imu() {
  double converted_gyro_x, converted_gyro_y, converted_gyro_z;         //Converted to °/s

  double angle_gyro_x, angle_gyro_y, angle_gyro_z;
  double total_acc;                                                    //Total force vector
  read_mpu_6050_data();                                                //Pull data from imu

  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro value

  converted_acc_x = acc_x / (double)ACCRATE;                           //Convert from raw value to m/s²
  converted_acc_y = acc_y / (double)ACCRATE;
  converted_acc_z = acc_z / (double)ACCRATE;

  converted_gyro_x = gyro_x / GYRORATE;                                //Convert from raw value to °/s
  converted_gyro_y = gyro_y / GYRORATE;
  converted_gyro_z = gyro_z / GYRORATE;

  total_acc = sqrt(converted_acc_x * converted_acc_x + converted_acc_y * converted_acc_y + converted_acc_z * converted_acc_z);        //Total acceleration vector

  acc_y_angle = acos(converted_acc_x/ total_acc) * 57.2958;                                                                         //Calculate angle between acceleration vector and total acceleration vector in °/s

  if (isnan(acc_y_angle)) acc_y_angle = 0;                                                                                      //Check for division by 0
  angle_y = alpha * (angle_y + converted_gyro_y * delta / MICROSECONDE) + (1 - alpha) * acc_y_angle;                            //Complementary filer
  corrected_angle = angle_y;

  //if(converted_acc_z < 0.075) corrected_angle *= -1;
}

void read_mpu_6050_data() {                                            //Reads raw gyro and accelerometer data
  int temperature;
  Wire.beginTransmission(MPUADRESS);                                   //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(MPUADRESS, 14);                                     //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers() {

  //Activate the MPU-6050
  Wire.beginTransmission(MPUADRESS);                                   //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission

  //Configure the accelerometer (+/-4g)
  //Register is 0x1C and works like the next one

  Wire.beginTransmission(MPUADRESS);                                   //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the register to +/- 4g
  Wire.endTransmission();                                              //End the transmission

  Wire.beginTransmission(MPUADRESS);                                   //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the register adress
  Wire.write(0x00);                                                    //Set the register to 250°/s
  Wire.endTransmission();                                              //End the transmission

  Wire.beginTransmission(MPUADRESS);                                   //Start communication with the address found during search
  Wire.write(0x1A);                                                    //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                              //End the transmission with the gyro
}

void calibrate_gyroscope() {
  for(int cal_int = 0; cal_int < CALITE; cal_int++) {                 //Run this code 2000 times

    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050

    gyro_y_cal += gyro_y;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    delay(4);                                                          //Delay 4us to simulate the 250Hz program loop
  }
  current = Quaternion(1, 0, 0);
  gyro_y_cal /= CALITE;                                                //Divide the gyro_x_cal variable by 2000 to get the average offset 
}
