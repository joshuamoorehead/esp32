//Copyright 2025 Joshua Moorehead
#include <math.h>
#include <stdint.h>

#define SCL 22
#define SDA 21
#define MPU6050_ADDR 0x68

#define power_mng_reg 0x6B
#define x_accel_h 0x3B
#define x_accel_l 0x3C
#define y_accel_h 0x3D
#define y_accel_l 0x3E
#define z_accel_h 0x3F
#define z_accel_l 0x40


void i2cWRITEBYTE(uint8_t data);
uint8_t i2cREADBYTE(uint8_t ack);
uint8_t i2cACK(uint8_t mode);
void pulseSCL();
void i2cSTART();
void i2cSTOP();
void writeRegister(uint8_t reg, uint8_t data);
uint8_t readRegister(uint8_t reg);


void setup() {
  Serial.begin(115200); // baud rate is 115,200 bits/s

  pinMode(SCL, OUTPUT_OPEN_DRAIN); // Clock pin
  pinMode(SDA, OUTPUT_OPEN_DRAIN);  // Data pin
  digitalWrite(SDA, LOW);

  writeRegister(power_mng_reg, 0x00);


}

void i2cSTART() {
  delayMicroseconds(1000);
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, LOW);
}

void i2cSTOP() {
  pinMode(SDA, OUTPUT_OPEN_DRAIN); //new
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH); // leave it high?
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);
  
}

void i2cWRITEBYTE(uint8_t data) {
  pinMode(SDA, OUTPUT_OPEN_DRAIN); // new?
    for(int i = 7; i>=0; i--) {
      digitalWrite(SCL,LOW); 
      delayMicroseconds(10);

      digitalWrite(SDA, (data & (1<<i))); //bit shifting
      delayMicroseconds(10);

      pulseSCL();
    }
}

uint8_t i2cREADBYTE () {
  pinMode(SDA, INPUT_PULLUP); // release sda to slave 
  delayMicroseconds(10);

  uint8_t data = 0;
  for(int i = 7; i>=0; i--) {
    digitalWrite(SCL,HIGH); //ready to read
    delayMicroseconds(10);

    data |= (digitalRead(SDA) << i); // bit shifting wth OR
    delayMicroseconds(10);

    digitalWrite(SCL,LOW);
    delayMicroseconds(10);
  }

  return data;

}

void pulseSCL() {
  digitalWrite(SCL,HIGH);
  delayMicroseconds(50);
  digitalWrite(SCL,LOW);
}


uint8_t i2cACK(uint8_t mode) {
  if(mode == 0) { // wait ack
    pinMode(SDA, INPUT_PULLUP); // release sda to slave 
    delayMicroseconds(10);
    digitalWrite(SCL,HIGH);
    delayMicroseconds(10);
    uint8_t response = 1;
    uint8_t ack = 1;
    /*for(int i = 0; i<7;i++) {
      if(!(response = digitalRead(SDA))) {
          ack = 0;
      }; // check NACK=HIGH  ACK=LOW from slave after rising clock
      pulseSCL();

    }*/
    
    /*uint8_t timeout = 1;
    while (timeout < 1000) {
      response = digitalRead(SDA);
      if(!response) {
        ack = 0;
        digitalWrite(SCL,LOW);
        pinMode(SDA, OUTPUT_OPEN_DRAIN);
        return ack;
      }
      timeout+=1;
    }*/

    for(int i = 0; i < 10000; i++) {
      if(!(response = digitalRead(SDA))) {
          ack = 0;
          digitalWrite(SCL,LOW);
          //pinMode(SDA, OUTPUT_OPEN_DRAIN); //Fine for write, keep input pull for a read ex
          return ack;
      }
    }
  
    //pinMode(SDA, OUTPUT_OPEN_DRAIN); // take back control of sda

    // if (ack == 1) {  //NACK, print an error
    //     Serial.println("ACK="); Serial.println(ack, DEC);
    // }
    Serial.println("ACK2="); Serial.println(ack, DEC);
    digitalWrite(SCL,LOW);
    return ack;
  } else { // send ack
      pinMode(SDA, OUTPUT_OPEN_DRAIN);

      digitalWrite(SDA, mode ? HIGH : LOW); 
      delayMicroseconds(100);

      pulseSCL();

      if(mode == 1) { // NACK
        pinMode(SDA, INPUT_PULLUP);
      }

      return 0;

  }
}

void writeRegister(uint8_t reg, uint8_t data) {
    Serial.print("Writing to Register 0x"); Serial.print(reg, HEX);
    Serial.print(" with Data 0x"); Serial.println(data, HEX);

    i2cSTART();

    i2cWRITEBYTE(MPU6050_ADDR << 1 | 0);  //Send device address + Write bit
    delayMicroseconds(100);
    if (i2cACK(0)) {
        Serial.println("NACK after device address (WRITE)!");
        i2cSTOP();
        return;
    }
    

    i2cWRITEBYTE(reg); // Send register address
    delayMicroseconds(100);
    if (i2cACK(0)) {
        Serial.println("NACK after register address!");
        i2cSTOP();
        return;
    }

    i2cWRITEBYTE(data); // Send data
    delayMicroseconds(100);
    if (i2cACK(0)) {
        Serial.println("NACK after data write!");
    }

    i2cSTOP();

    //Verify the write was successful
    uint8_t verify = readRegister(reg);
    Serial.print("Read Back Register 0x"); Serial.print(reg, HEX);
    Serial.print(": 0x"); Serial.println(verify, HEX);

    if (verify != data) {
        Serial.println("⚠️ WARNING: Register write failed!");
    }
}


uint8_t readRegister(uint8_t reg) {
  Serial.print("Reading Register 0x"); Serial.println(reg, HEX);

  i2cSTART();

  i2cWRITEBYTE(MPU6050_ADDR << 1 | 0);
  delayMicroseconds(100);
  if (i2cACK(0)) {
    Serial.println("NACK after device address (WRITE)!");
    i2cSTOP();
    return 0;
  }

  i2cWRITEBYTE(reg);
  delayMicroseconds(100);
  if (i2cACK(0)) {
    Serial.println("NACK after register address!");
    i2cSTOP();
    return 0;
  }

  i2cSTOP(); // end transaction 1
  delayMicroseconds(50); 
  i2cSTART(); // start transaction 2

  i2cWRITEBYTE(MPU6050_ADDR << 1 | 1);
  delayMicroseconds(100);
  if (i2cACK(0)) {
    Serial.println("NACK after device address (READ)!");
    i2cSTOP();
    return 0;
  }

  uint8_t data = i2cREADBYTE();
  //i2cACK(0); // 1 = send NACK
  Serial.print("Read Data: 0x"); Serial.println(data, HEX);

  i2cSTOP();

  return data;
}

void calculateTiltAngles(int16_t ax, int16_t ay, int16_t az) {
    
    float tiltXY = atan2(ay, ax) * 180.0 / M_PI; //TODO use rad to deg thangalang
    float tiltXZ = atan2(az, ax) * 180.0 / M_PI;
    float tiltYZ = atan2(az, ay) * 180.0 / M_PI;

    Serial.print("XY Tilt: "); Serial.print(tiltXY);
    Serial.print(" | XZ Tilt: "); Serial.print(tiltXZ);
    Serial.print(" | YZ Tilt: "); Serial.println(tiltYZ);
}

void loop() {

    Serial.println("\n--- Reading Acceleration Data ---");

    int16_t ax = (readRegister(x_accel_h) << 8) | readRegister(x_accel_l);
    int16_t ay = (readRegister(y_accel_h) << 8) | readRegister(y_accel_l);
    int16_t az = (readRegister(z_accel_h) << 8) | readRegister(z_accel_l);

    //convert raw values to `g` using the scale factor (default: ±2g, 16384 LSB/g)
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    //acceleration values in `g`
    Serial.print("Accel X: "); Serial.print(accelX, 3);  // 3 decimal places
    Serial.print(" g | Y: "); Serial.print(accelY, 3);
    Serial.print(" g | Z: "); Serial.println(accelZ, 3);

    calculateTiltAngles(ax, ay, az);


    delay(1000);
  

}
