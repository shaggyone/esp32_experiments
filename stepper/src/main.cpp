// https://gist.github.com/metalinspired/dcfe07ed0b9f42870eb54dcf8e29c126
// #include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

#define RX2 16
#define TX2 17
// #define DIAG_PIN 15         // STALL motor 2
#define EN_PIN 21           // Enable
#define DIR_PIN 23          // Direction
#define STEP_PIN 22         // Step
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // Sernsor resistor R_SENSE for current calc.
// #define STALL_VALUE 2       // [0..255]
#define STALL_VALUE 100       // [0..255]

// / Select your stepper driver type
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware Serial
//TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
// TMC2209Stepper driver(RX2, TX2, R_SENSE, DRIVER_ADDRESS);

#define MICROSTEPS 16

constexpr uint32_t steps_per_mm = 16 * MICROSTEPS;

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);         // Init serial port and set baudrate

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  // digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // SERIAL_PORT.begin(115200, SERIAL_8N1, RX2, TX2);
  SERIAL_PORT.begin(115200);      // HW UART drivers
//driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(4);                 // Enables driver in software
  // driver.toff(5);              // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(MICROSTEPS);          // Set microsteps to 1/16th

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  driver.blank_time(24);

  // driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);        // Needed for stealthChop

  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driver.TCOOLTHRS(0xFFFFF); // 20bit max

  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  driver.semin(5);

  // CoolStep upper threshold [0... 15].
  // If SG is sampled equal to or above this threshold enough times,
  // CoolStep decreases the current to both coils.
  driver.semax(2);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  driver.sedn(0b01);

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  driver.SGTHRS(STALL_VALUE);

  delay(100);
  Serial.print("\nTesting connection...");
  uint8_t result = driver.test_connection();

  if (result) {
    Serial.println("failed!");
    Serial.print("Likely cause: ");

    switch(result) {
      case 1: Serial.println("loose connection"); break;
      case 2: Serial.println("no power"); break;
    }

    Serial.println("Fix the problem and reset board.");

    // We need this delay or messages above don't get fully printed out
    delay(1000);
    abort();
  }

  Serial.println("OK");


  stepper.setMinPulseWidth(50);
  stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(200*steps_per_mm); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}

int sign = 1;

void loop() {
  // // Run 5000 steps and switch direction in software
  // for (uint16_t i = 5000; i>0; i--) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(160);
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(160);
  // }
  // shaft = !shaft;
  // driver.shaft(shaft);

  if (stepper.distanceToGo() == 0) {
    // stepper.disableOutputs();
    delay(1000);
    signed long int rotate_distance = sign * 5 * 200 * MICROSTEPS;
    Serial.printf("Rotating %d steps\n", rotate_distance);

    stepper.move(rotate_distance); // Move 100mm
    // stepper.enableOutputs();
    sign = -sign;
  }

  stepper.run();
}