#include <Wire.h>
#include <Adafruit_NeoPixel.h>

/* HAPTIC DRIVER */
// Adafruit_DRV2605 drv;

/* MPU PARAMETERS
* 1. Variables for storing Data
* 2. Reading Measurements
* 3. Power
*/

// MPU6050 mpu;
float Wx, Wy, Wz; // Roll, pitch, yaw
float Ax, Ay, Az; 
float angleRoll, anglePitch;
uint32_t loopTimer;

// gyroscope config
#define LOW_PASS_REG 0x1A
#define DLPF_CFG 5 // Digital Low pass filter Config
#define MPU_I2C_ADDR 0x68
#define GYRO_CFG_REG 0x1B
#define GYRO_LSB 65.5
#define GFS_SEL 8
#define GYRO_HEAD_REG 0x43
#define PWR_MGMT_1 0x6B
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// accel config
#define ACC_CFG_REG 0x1C
#define AFS_SEL 0x8 // full-scale range += 4g -> 8192 or 4096 LSB
#define ACC_HEAD_REG 0x3B
#define ACC_LSB 8192
#define ax_offset -0.04
#define ay_offset 0.02
#define az_offset -0.15
/*TIMING MACROS*/
#define dT 0.004 // 4ms loop period -> 250Hz

/* I2C COMMUNICATION MACROS */
#define SDA_PIN 8
#define SCL_PIN 9
#define CLOCK_FREQ 400000 

/* DATA COLLECTION MACROS*/
int currentLabel = 0;


/* Kalman Filter Macros */
float angleRollK = 0, angleRollErr = 2*2; // initial angle
float anglePitchK = 0, anglePitchErr = 2*2;

float kalmanOutput[2] = {0,0}; // predicted angle, uncertainty

void kalman_1d(float state, float uncertainty, float input, float measurement){
  state += dT *input; // predict current state
  uncertainty += dT * dT * 4 * 4; // calculate uncertainty of prediction
  float gain = uncertainty * 1/(uncertainty + 3 * 3); // calc gain
  state += gain *(measurement - state); // update predicted state
  uncertainty = (1-gain) * uncertainty; // update uncertainty of predicted state

  kalmanOutput[0] = state;
  kalmanOutput[1] = uncertainty;
}

void apply_kalman(){
  kalman_1d(angleRollK, angleRollErr, Wx, angleRoll);
  angleRollK = kalmanOutput[0];
  angleRollErr = kalmanOutput[1];

  kalman_1d(anglePitchK, anglePitchErr, Wy, anglePitch);
  anglePitchK = kalmanOutput[0];
  anglePitchErr = kalmanOutput[1];
}

/* User-defined functions */
void i2c_write(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) {
    Serial.println("i2c_write: write failed");
    while (1);
  }
}
void i2c_readN(uint8_t addr, uint8_t startReg, uint8_t n, uint8_t *dest){
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission() != 0) {
    Serial.println("i2c_readN: read failed");
    while (1);
  }

  uint8_t received = Wire.requestFrom(addr, n);
  if (received != n) {
    Serial.print("i2c_readN: expected ");
    Serial.print(n);
    Serial.print(" bytes, got ");
    Serial.println(received);
    while (1);
  }
  for (uint8_t i = 0; i < n; i++){
    if (Wire.available()){
      dest[i] = Wire.read();
    } else {
      Serial.println("Wire not available, waiting to read next byte");
      while (1);
    }
  };
}
void config_imu(){
  i2c_write(MPU_I2C_ADDR, LOW_PASS_REG, DLPF_CFG); // Set configuration for LPF
  i2c_write(MPU_I2C_ADDR, GYRO_CFG_REG, GFS_SEL); // Set gyroscope full scale
  i2c_write(MPU_I2C_ADDR, ACC_CFG_REG, AFS_SEL); // set full scale range
}
void read_gyro(void){

  uint8_t raw_gyro[6]; // read gyro measurements
  i2c_readN(MPU_I2C_ADDR, GYRO_HEAD_REG, 6, raw_gyro);

  int16_t raw_wx = raw_gyro[0]<<8 | raw_gyro[1];
  int16_t raw_wy = raw_gyro[2]<<8 | raw_gyro[3];
  int16_t raw_wz = raw_gyro[4]<<8 | raw_gyro[5];

  Wx = (float)raw_wx/GYRO_LSB; // convert into deg/s
  Wy = (float)raw_wy/GYRO_LSB;
  Wz = (float)raw_wz/GYRO_LSB;

}

void read_accel(void){
  uint8_t raw_accel[6];
  i2c_readN(MPU_I2C_ADDR, ACC_HEAD_REG, 6, raw_accel);
  int16_t raw_ax = raw_accel[0]<<8 | raw_accel[1];
  int16_t raw_ay = raw_accel[2]<<8 | raw_accel[3];
  int16_t raw_az = raw_accel[4]<<8 | raw_accel[5];

  Ax = (float)raw_ax/ACC_LSB; // convert into rad/s
  Ay = (float)raw_ay/ACC_LSB;
  Az = (float)raw_az/ACC_LSB;
}

void calc_angles(void){
  angleRoll = atan2f(Ay, sqrtf(Ax*Ax + Az*Az))/3.1416*180;
  anglePitch = atan2f(-Ax, sqrtf(Ay*Ay + Az*Az))/3.1416*180;
}


void initialize_imu(){
  i2c_write(MPU_I2C_ADDR, PWR_MGMT_1, 0);
  Serial.println("MPU6050 wake-up command sent");
  
  if (Wire.endTransmission() != 0){ // check for connection
    Serial.println("MPU6050 Connection Failed.");
    while(1);
  }
  config_imu();
  Serial.println("MPU6050 config complete");

}
void initialize_i2c_bus(){
  Wire.begin(SDA_PIN, SCL_PIN); //start bus
  Wire.setClock(CLOCK_FREQ);
  delay(250);
}
void print_gyro(){
  Serial.print("Roll rate [deg/s]=");
  Serial.print(Wx);
  Serial.print("Pitch rate [deg/s]=");
  Serial.print(Wy);
  Serial.print("Yaw rate [deg/s]=");
  Serial.print(Wz);
  Serial.print("\n");
}
void print_accel(){
  Serial.print("Ax =");
  Serial.print(Ax);
  Serial.print("Ay =");
  Serial.print(Ay);
  Serial.print("Az =");
  Serial.println(Az);
}
void print_angles(){
  Serial.print("Roll_angle:");
  Serial.print(angleRoll);
  Serial.print(",");
  Serial.print("Pitch_angle:");
  Serial.println(anglePitch);
}
void apply_offsets(){
  Wx -= gx_offset;
  Wy -= gy_offset;
  Wz -= gz_offset;

  Ax += ax_offset;
  Ay += ay_offset;
  Az += az_offset; 
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  // pixels.begin();
  // pixels.clear();
  // pixels.show();
  Serial.println("Initating I2C Bus...");
  initialize_i2c_bus();
  
  Serial.println("Initating MPU6050...");
  initialize_imu();
  
  Serial.println("Determining static offsets...");
  for(int i = 0; i < 2000; i++){ // remove biases
    read_gyro();
    read_accel();
    if (i % 500 == 0) {
      Serial.print("Offset sample ");
      Serial.println(i);
    }
    gx_offset += Wx;
    gy_offset += Wy;
    gz_offset += Wz;
    delay(1);    
  }
  gx_offset /= 2000;
  gy_offset /= 2000;
  gz_offset /= 2000;

  Serial.println("Finished determining offset.");
  delay(250);
  loopTimer = millis();
}
void print_kalman(){
  Serial.print("Roll_angle_K:");
  Serial.print(angleRollK);
  Serial.print(",");
  Serial.print("Pitch_angle_K:");
  Serial.println(anglePitchK);
}

void loop() {
  // driver_loop();
  read_gyro();
  read_accel();
  apply_offsets();
  calc_angles();
  apply_kalman();
  print_kalman();

  
  // print_gyro();
  // Serial.println("----------------------");
  // print_accel(f);
  // Serial.println("----------------------");
  // print_angles();  
  while (millis() - loopTimer < (dT*1000));
  loopTimer = millis();
}
