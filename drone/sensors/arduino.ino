#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_VL53L0X.h>

/* ================= Ultrasonic I2C ================= */
#define SensorAddress byte(0x70)
#define RangeCommand  byte(0x51)

/* ================= Objects ================= */
Adafruit_MPL3115A2 mpl;
Adafruit_VL53L0X lox;

/* ================= Timing ================= */
unsigned long now;

/* Ultrasonic */
unsigned long usLastTrigger = 0;
const unsigned long usInterval = 60;
uint16_t lastUSmm = 0;

/* VL53L0X */
unsigned long vlLastRead = 0;
const unsigned long vlInterval = 50;
uint16_t lastVLmm = 0;

/* MPL3115A2 */
enum MPLState { MPL_IDLE, MPL_WAIT };
MPLState mplState = MPL_IDLE;
unsigned long mplLastTrigger = 0;
const unsigned long mplInterval = 10;
int32_t lastMPLmm = 0;

/* ================= Setup ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  /* MPL3115A2 */
  mpl.begin();
  // mpl.setMode(MPL3115A2_ALTIMETER);
  mpl.setSeaPressure(1013.26);
  mpl.setAltitudeOffset(-mpl.getAltitude());
  /* VL53L0X */
  lox.begin();
  lox.startRangeContinuous();
}

/* ================= Loop ================= */
void loop() {
  now = millis();

  handleUltrasonic();
  handleVL53L0X();
  handleMPL3115A2();
}

/* ================= Publishing ================= */
void publish(char name, uint8_t status, int32_t value) {
  Serial.print(name);
  Serial.print(',');
  Serial.print(status);
  Serial.print(',');
  Serial.println(value);
}

/* ================= Ultrasonic ================= */
void handleUltrasonic() {
  if (now - usLastTrigger >= usInterval) {
    usLastTrigger = now;
    takeRangeReading();
  }

  uint16_t rangeCm = requestRange();
  if (rangeCm > 0) {
    uint16_t mm = rangeCm * 10;
    lastUSmm = mm;
    if (mm == 7200) {
      publish('G', 0, mm);
    } else {
      publish('G', 1, mm);
    }
    
  }
}

void takeRangeReading() {
  Wire.beginTransmission(SensorAddress);
  Wire.write(RangeCommand);
  Wire.endTransmission();
}

uint16_t requestRange() {
  Wire.requestFrom(SensorAddress, byte(2));
  if (Wire.available() >= 2) {
    byte highByte = Wire.read();
    byte lowByte  = Wire.read();
    return word(highByte, lowByte);
  }
  return 0;
}

/* ================= VL53L0X ================= */
void handleVL53L0X() {
  if (now - vlLastRead < vlInterval) return;
  vlLastRead = now;
  if (lox.isRangeComplete()) {
    uint16_t mm = lox.readRange();
    if (mm == 8191) {
      publish('V', 0, mm);
    } else {
      publish('V', 1, mm);
    }
    lastVLmm = mm;
  }
}

/* ================= MPL3115A2 ================= */
void handleMPL3115A2() {
  switch (mplState) {
    case MPL_IDLE:
      if (now - mplLastTrigger >= mplInterval) {
        mplLastTrigger = now;
        mpl.startOneShot();
        mplState = MPL_WAIT;
      }
      break;

    case MPL_WAIT:
      if (mpl.conversionComplete()) {
        float alt_m = mpl.getLastConversionResults(MPL3115A2_ALTITUDE);
        if (!isnan(alt_m)) {
          int32_t mm = alt_m * 1000.0f;
          publish('B', 1, mm);          
        }
        mplState = MPL_IDLE;
      }
      break;
  }
}
