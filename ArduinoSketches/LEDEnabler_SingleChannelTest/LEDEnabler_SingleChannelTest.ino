#include <Wire.h>

const int VER = 0.70;

TaskHandle_t MainTask;
TaskHandle_t StatusTask;
TaskHandle_t WebserverTask;

bool DEBUG = true;            // Used to enable verbose serial logging, to aid in debugging

bool errorDisplay = true;     // Whether or not the LED is used to display any errors
const int errorLEDpin = 2;    // GPIO 2 is the onboard LED
int errorType = 0;            // 0 for no errors, 1 for i2c error (missing ADC, etc.), 2 for ??????

const int I2C_SDA = 21;       // GPIO 21, pin 11
const int I2C_SCL = 22;       // GPIO 22, pin 14
const int PWM_FREQ = 40000;   // 40kHz, beyond audibility (not that that really matters in this application)
const int PWM_RESOLUTION = 8;

const int BULB_VOLTAGE_THRESH = 100; // Arbitrary number somewhere between 0 and 2048 to mark where bulb is on (after division from 12V)

const int pwmPins[11] = {25, 26, 27, 18, 19, 16, 17, 32, 33, 13, 14}; // L Brake, R Brake, Third Brake, L Rev., R Rev., L Turn, R Turn, L Tail, R Tail, L Fog, R Fog, in order of ADC channel
int pwmStatus[11] = {0};      // Used to store current status of each PWM channel
                              // PWM channels are in order from 0 to 10, so no variable map is required

const int DIVIDER_SCALING = 10;   // Resistive divider is 10:1 to scale ~12V level down into range the ADC will measure
const int RESISTANCE[11] = {50};  // Resistance in ohms of the power resistor
const int PWR_TARGET[11] = {21, 21, 15, 21, 21, 21, 21, 5, 5, 5, 5}; // Target Wattage to hit when adjusting PWM

const byte adcAddr = 0b0110101;

// Set up csel bits for reading all channels of ADC
const byte ch0 = 0b00000000;
const byte ch1 = 0b00000010;
const byte ch2 = 0b00000100;
const byte ch3 = 0b00000110;
const byte ch4 = 0b00001000;
const byte ch5 = 0b00001010;
const byte ch6 = 0b00001100;
const byte ch7 = 0b00001110;
const byte ch8 = 0b00010000;
const byte ch9 = 0b00010010;
const byte ch10 = 0b00010100;

const byte chArray[] = {ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10}; 

// TwoWire I2CADC = TwoWire(0);

void setup() {

  Serial.begin(115200);
  Serial.print(F("Beginning Saab LED Enabler v"));
  Serial.println(VER);

  pinMode(errorLEDpin, OUTPUT);

  for (int i = 0; i <= 11; i++) {
    ledcSetup(i, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pwmPins[i], i);
  }
  Serial.println(F("PWM channels initialized"));

  Wire.begin();
  Serial.println(F("I2C initialized."));

  xTaskCreatePinnedToCore(Task1_Main, "Main Loop", 10000, NULL, 1, &MainTask, 0);
  xTaskCreatePinnedToCore(Task2_Status, "Status Blinker", 10000, NULL, 1, &StatusTask, 1);

  Serial.println(F("setup() completed."));
  
}

void Task1_Main(void * pvParameters) {
  for (;;) {
    int values[11];

    Serial.println("Attempting to read from I2C...");
    bool i2cReadStatus = scanRead(values); // Pass array into function

    if (!i2cReadStatus) {
      errorType = 1; // Update error flag to error function to flash the code
      Serial.println("I2C Error, doing nothing.");

    } else { // Otherwise, continue 
      int currentIndex = 5;
      int channelDutyCycle = 127;

      // For now, we're only paying attention to a single channel, to prove the concept.
      // Left Turn 
      if (values[currentIndex] > BULB_VOLTAGE_THRESH) {
        if (DEBUG) {
          Serial.println("Left Turn signal is on");
        }
        if (pwmStatus[currentIndex] == 0) { // If PWM is not on, meaning the light just turned on

          // Turn on PWM
          ledcWrite(currentIndex, channelDutyCycle);

          pwmStatus[currentIndex] = 1;      // Mark that it's enabled
        }
        else { // PWM was already on, we're just going to tweak the PWM duty cycle to try and nail the power
          channelDutyCycle = calcDutyCycle(currentIndex, values);

        }
      } else if (values[currentIndex] <= BULB_VOLTAGE_THRESH) { // Bulb is now off
        if (DEBUG) {
          Serial.println("Left Turn signal is now off");
        }
        if (pwmStatus[currentIndex] == 0) { // PWM is off currently
          // We're good, nothing required here
        } else {
          ledcWrite(currentIndex, 0);       // Set duty cycle to 0, turning PWM off

          pwmStatus[currentIndex] = 0;      // Mark that it's off
        }

      }
    }

  } // End for(;;)
  
} // End Task1_Main()

void Task2_Status(void * pvParameters) {
  for (;;) {
    switch (errorType) {
      case 1:
        digitalWrite(errorLEDpin, HIGH);
        delay(200);
        digitalWrite(errorLEDpin, LOW);
        delay(100);
        digitalWrite(errorLEDpin, HIGH);
        delay(200);
        digitalWrite(errorLEDpin, LOW);
        delay(1000);
        break;

      default:
      // No error, force LED off to indicate no errors
        digitalWrite(errorLEDpin, LOW);
        break;
    }
  } // End for(;;)
  
} // End Task2_Error()


void loop() {

}


bool scanRead(int channelData[]) {
  bool status = false; // Flag to keep track of I2C communication status
  byte i2cData[22];

  int returnBytes = Wire.requestFrom(adcAddr, 22);
  if (returnBytes > 0) {
    status = true; // Got data, can continue
    Serial.println("Some data exists, starting read...");

    int i = 0;
    while (Wire.available()) {
      i2cData[i] = Wire.read();
      i++;
    }
    Serial.println("Read complete, starting calculations...");

    // Now we have all the data, convert it into 'real' numbers
    for (int ii = 0; i <= 11; i++) { // Loop through all channels (0 through 10, 11 total)
      int msbIndex = ii * 2;
      int lsbIndex = msbIndex + 1;
      byte realMSB = 0x00000011 & i2cData[msbIndex]; // This zeros out the first 6 bits of the MSB byte (1's from the ADC)
      if (DEBUG) {
        Serial.print("realMSB: ");
        Serial.println(realMSB, BIN);
      }
      int combined;
      combined = realMSB;
      combined = combined * 256;
      combined |= i2cData[lsbIndex];
      channelData[ii] = combined;

      if (DEBUG) {
        Serial.print("Channel ");
        Serial.print(ii);
        Serial.print(" ADC Value: ");
        Serial.println(combined);
      }
    }

  } else {
    status = false; // No data, notify the main loop
    Serial.println("No data received, reporting status of false");
  }

  return status;
}

int calcDutyCycle(int inputChannel, int values[]) {
  int voltageValue = values[inputChannel] * DIVIDER_SCALING;

  int dutyRequired = (PWR_TARGET[inputChannel] * RESISTANCE[inputChannel] ) / (voltageValue * voltageValue);

  return dutyRequired;
}

