#include <Wire.h>

const int VER = 0.80;

TaskHandle_t MainTask;
TaskHandle_t StatusTask;
TaskHandle_t WebserverTask;

bool DEBUG = true;  // Used to enable verbose serial logging, to aid in debugging
bool ADC_DEBUG = true;
bool CALIBRATION = true;  // Used to change behavior of main loop, to understand ADC values for given voltages

bool errorDisplay = true;   // Whether or not the LED is used to display any errors
const int errorLEDpin = 2;  // GPIO 2 is the onboard LED
int errorType = 0;          // 0 for no errors, 1 for i2c error (missing ADC, etc.), 2 for ??????

const int I2C_SDA = 21;      // GPIO 21, pin 11
const int I2C_SCL = 22;      // GPIO 22, pin 14
const int PWM_FREQ = 40000;  // 40kHz, beyond audibility (not that that really matters in this application)
const int PWM_RESOLUTION = 8;

const int BULB_VOLTAGE_THRESH = 400;  // Arbitrary number somewhere between 0 and 2048 to mark where bulb is on (after division from 12V)

const int pwmPins[11] = { 25, 26, 27, 18, 19, 16, 17, 32, 33, 13, 14 };  // L Brake, R Brake, Third Brake, L Rev., R Rev., L Turn, R Turn, L Tail, R Tail, L Fog, R Fog, in order of ADC channel
int pwmStatus[11] = { 0 };                                               // Used to store current status of each PWM channel
                                                                         // PWM channels are in order from 0 to 10, so no variable map is required

const int DIVIDER_SCALING = 10;                                         // Resistive divider is 10:1 to scale ~12V level down into range the ADC will measure
const int RESISTANCE[11] = { 50 };                                      // Resistance in ohms of the power resistor
const int PWR_TARGET[11] = { 21, 21, 15, 21, 21, 21, 21, 5, 5, 5, 5 };  // Target Wattage to hit when adjusting PWM

const byte ADC_ADDR = 0b0110101;
const byte ADC_SETUP = 0b11010010;  //Register, Sel2, Sel1, Sel0, Clk, Bip/Uni, Reset, DC
  // Setup byte, Internal reference, ref output, internal ref on, nternal clock, unipolar, no reset, don't care
const byte ADC_CONFIG = 0b00010101;  // Register, Scan1, Scan0, CS3, CS2, CS1, CS0, SGL/DIF
  // Config byte, Scan up, Channel 10, Single ended


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
  Serial.println(F("I2C initialized, attempting to configure ADC..."));
  adcSetup(ADC_SETUP);
  adcConfigure(ADC_CONFIG);

  xTaskCreatePinnedToCore(Task1_Main, "Main Loop", 10000, NULL, 1, &MainTask, 0);
  xTaskCreatePinnedToCore(Task2_Status, "Status Blinker", 10000, NULL, 1, &StatusTask, 1);

  Serial.println(F("setup() completed."));
}

void Task1_Main(void* pvParameters) {
  for (;;) {
    uint16_t values[10] = {};  // 10 channels

    if (CALIBRATION) {
      Serial.println("Reading ADC values via I2C in 5 seconds");
      delay(5000);
    }

    Serial.println("Attempting to read ADC value via I2C...");
    scanRead(values);  //Results are stored in values
    Serial.println("Calibration values:");

    if (CALIBRATION) {
      int indexToCal = 5;
      
      for (int calChannel = 0; calChannel < 10; calChannel++) {
        Serial.print("Current value of channel ");
        Serial.print(calChannel+1);
        Serial.print(": ");
        Serial.println(values[calChannel]);
      }
      
    } else {  // Normal program goes here

      int currentIndex = 5;
      int channelDutyCycle = 127;

      // For now, we're only paying attention to a single channel, to prove the concept.
      // Left Turn
      if (values[currentIndex] > BULB_VOLTAGE_THRESH) {
        if (DEBUG) {
          Serial.println("Left Turn signal is on");
        }
        if (pwmStatus[currentIndex] == 0) {  // If PWM is not on, meaning the light just turned on

          // Turn on PWM
          ledcWrite(currentIndex, channelDutyCycle);

          pwmStatus[currentIndex] = 1;  // Mark that it's enabled
        } else {                        // PWM was already on, we're just going to tweak the PWM duty cycle to try and nail the power
          channelDutyCycle = calcDutyCycle(currentIndex, values);
        }
      } else if (values[currentIndex] <= BULB_VOLTAGE_THRESH) {  // Bulb is now off
        if (DEBUG) {
          Serial.println("Left Turn signal is now off");
        }
        if (pwmStatus[currentIndex] == 0) {  // PWM is off currently
          // We're good, nothing required here
        } else {
          ledcWrite(currentIndex, 0);  // Set duty cycle to 0, turning PWM off

          pwmStatus[currentIndex] = 0;  // Mark that it's off
        }
      }
    }    // End if(CALIBRATION)/else block

  }  // End for(;;)

}  // End Task1_Main()

void Task2_Status(void* pvParameters) {
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
  }  // End for(;;)

}  // End Task2_Error()


void loop() {
}


void scanRead(uint16_t *channelData) {
  adcConfigure(ADC_CONFIG); 

  Wire.requestFrom(ADC_ADDR, 20, false);

  if (Wire.available() == 20) {
    for (uint8_t i = 0; i < 10; i++) {  // Read all 10 channels
      *(channelData + i) = (Wire.read() & 0x03) << 8;  // MSB is first, bit 7 through 2 are high
      *(channelData + i) |= Wire.read() & 0x00ff;  // Read LSB

    }
  }
  Serial.println("Read complete...");

}

void adcSetup(byte setupByte) {
  Wire.beginTransmission(ADC_ADDR);
  Wire.write(setupByte);
  Wire.endTransmission();
}

void adcConfigure(byte configByte) {
  Wire.beginTransmission(ADC_ADDR);
  Wire.write(configByte);
  Wire.endTransmission();
}

int calcDutyCycle(int inputChannel, uint16_t *values) {
  int voltageValue = values[inputChannel] * DIVIDER_SCALING;

  int dutyRequired = (PWR_TARGET[inputChannel] * RESISTANCE[inputChannel]) / (voltageValue * voltageValue);

  return dutyRequired;
}
