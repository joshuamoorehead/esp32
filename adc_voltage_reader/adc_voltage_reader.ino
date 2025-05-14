// Copyright 2025 Joshua Moorehead
#define ANALOG_PIN 34
#define DELAY_TIME 500
#define NUM_SEGMENTS 16  //Divide into 16 segments
#define NUM_SAMPLES 285  // (14.2V / 0.05) + 1 =  steps
#define R1 33.0
#define R2 10.0
double calibrationData[NUM_SAMPLES][2];  // Column 0: Voltage, Column 1: ADC Value
double m_values[NUM_SEGMENTS];  // Slopes for each segment
double b_values[NUM_SEGMENTS];  // Intercepts for each segment

double predictADC(int analogV) {
   int segment = analogV / 256;
   //Serial.printf("Segment: %d\n", segment);
   return m_values[segment] * analogV + b_values[segment];
}

void monitoring() {
    //MONITORING
    //Print out raw voltage, ADC read value, and corrected voltage by feeding read voltage through model (16 segment arrays for m and b). Print out corrected voltage*43/10 for the actual battery voltage
    int rawADC = analogRead(ANALOG_PIN);
    double rawVoltage = (rawADC/4095.0) * 3.3; // Equation to help change our ADC Read into a raw voltage
    //double correctedVoltage = (predictADC(rawADC)/4095.0) * 3.3;
    double correctedVoltage = (predictADC(rawADC)) * 10.0/43.0; //TODO Bad for business
    /*if(correctedVoltage >= 3.3) {
      correctedVoltage = 3.3;
    }*/
    double actualVoltage = correctedVoltage * ((R1 + R2) / R2);
    Serial.printf("Raw Voltage: %.2f [ADC read as %d]", rawVoltage, rawADC);
    Serial.printf(", Corrected voltage: %.2f,", correctedVoltage);
    Serial.printf(" Actual Voltage: %.2f\n", actualVoltage);
    delayMicroseconds(1000000);
    //TODO raw ADC and actual voltage are hovering relatively good, however the corrected Voltage is in the thousands, I am passing the raw voltage through it, dont know if I should be passing the
    //ADC value in instead, either a problem with the piecewise function or a problem with the way voltage is stored in calibration data
}

void printMessage() {
          Serial.println("Entering auto calibration mode...please wait.\n");
          //CALIBRATION
          Serial.println("Taking control of power supply...success\n");
          Serial.println("Autocalibrating\n");
}






void setup() {
  Serial.begin(115200);
  pinMode(34, INPUT);
  Serial2.begin(9600, SERIAL_8N1, 23, 19);

  printMessage();

  //Initiliaze connection with power supply
  Serial2.print("*IDN?\r\n"); //identification
  delay(5);
  Serial2.print("SYST:REM\r\n"); //remote mode
  delay(5);
  Serial2.print("INST:NSEL 1\r\n"); //selecting channel
  Serial2.print("OUTP 1\r\n");

  double voltage = 0;
  //Calibrate ADC with for loop incrementing power supply remotely by 50 mV from 0-3.3V (delay(500) between increments). Serial2.print("volt .5") = how to send voltage to power supply

  for (int i = 0; i < NUM_SAMPLES; i++) {
        if(i == 0) {
          Serial.println("Entering auto calibration mode...please wait.\n");
          //CALIBRATION
          Serial.println("Taking control of power supply...success\n");
          Serial.println("Autocalibrating\n");
        }
        Serial2.printf("VOLT %f\r\n", voltage);
        delay(DELAY_TIME);  // Wait 500ms before the next increment

        double ADCValue = analogRead(ANALOG_PIN); 
        //Serial.println(ADCValue);

        // Store values in 2D array
        calibrationData[i][1] = voltage;   // Store voltage (0-3.3V)
        calibrationData[i][0] = ADCValue;  // Store ADC reading (0-4095)

        voltage += 0.05;  // Increment voltage by 50mV
        if(i == 28) {
          Serial.println("10% complete");
        } else if (i == 28*2) {
          Serial.println("20% complete");
        } else if (i == 28*3) {
          Serial.println("30% complete");
        } else if (i == 28*4) {
          Serial.println("40% complete");
        } else if (i == 28*5) {
          Serial.println("50% complete");
        } else if (i == 28*6) {
          Serial.println("60% complete");
        } else if (i == 28*7) {
          Serial.println("70% complete");
        } else if (i == 28*8) {
          Serial.println("80% complete");
        } else if (i == 28*9) {
          Serial.println("90% complete");
        } else if (i == 28*10) {
          Serial.println("100% complete");
        }

    }
  Serial2.print("SYST:LOC\r\n");
  //Use stored ADV values (in array) to train PWL model
  //n=18 except for last one
  for(int i = 0; i<15; i++) {
    double xiyi = 0;
    double xi = 0;
    double yi = 0;
    double xi2 = 0;
    double b = 0;
    double m = 0;
    
    for(int j = 0; j<18; j++) {
      xiyi += calibrationData[(i*18) + j][0]*calibrationData[(i*18) + j][1];
      xi += calibrationData[(i*18) + j][0];
      yi += calibrationData[(i*18) + j][1];
      xi2 += calibrationData[(i*18) + j][0]*calibrationData[(i*18) + j][0];
    }
    m = ((18*xiyi)-(xi*yi))/((18*xi2)-(xi*xi));
    b = ((yi*xi2)-(xi*xiyi))/((18*xi2)-(xi*xi));
    m_values[i] = m;
    b_values[i] = b;
    //Serial.printf("y = %fx + %f\n", m, b);
  }
  // for last piece wise
  double xiyi = 0;
  double xi = 0;
  double yi = 0;
  double xi2 = 0;
  double b = 0;
  double m = 0;
  for(int j = 0; j<15; j++) {
    xiyi += calibrationData[(15*18) + j][0]*calibrationData[(15*18) + j][1];
    xi += calibrationData[(15*18) + j][0];
    yi += calibrationData[(15*18) + j][1];
    xi2 += calibrationData[(15*18) + j][0]*calibrationData[(15*18) + j][0];
  }
  //m = ((15*xiyi)-(xi*yi))/((15*xi2)-(xi*xi));
  //b = ((yi*xi2)-(xi*xiyi))/((15*xi2)-(xi*xi));

  m = ((15*xiyi)-(xi*yi))/((15*xi2)-(xi*xi));
  b = ((yi*xi2)-(xi*xiyi))/((15*xi2)-(xi*xi));
  m_values[15] = m;
  b_values[15] = b;
  //Serial.printf("y = %fx + %f\n", m, b);

  Serial.println("Entering MONITORING MODE!\n");
  
}

void loop() {
  monitoring();
}

/*for(int j = 0; j<15; j++) {
    xiyi += calibrationData[(16*17) + j][0]*calibrationData[j*16][1];
    xi += calibrationData[j*16][0];
    yi += calibrationData[j*16][1];
    xi2 += calibrationData[j*16][0]*calibrationData[j*16][0];
  }*/
