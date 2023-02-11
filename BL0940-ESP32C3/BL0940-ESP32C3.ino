// Need this for the lower level access to set them up.
#include <HardwareSerial.h>

//Define two Serial devices mapped to the two internal UARTs
HardwareSerial MySerial0(0);

const uint16_t timeout = 1000;  //Serial timeout[ms]
const float Vref = 1.218;       //[V]
const float R5 = 3.3;           //[Ohm]
const float Rt = 2000.0;        //n:1
const float R8 = 20.0;          //[kOhm]
const float R9 = 20.0;          //[kOhm]
const float R10 = 20.0;         //[kOhm]
const float R11 = 20.0;         //[kOhm]
const float R12 = 20.0;         //[kOhm]
const float R7 = 24.9;          //[Ohm]
uint16_t Hz = 60;               //[Hz]
uint16_t updateRate = 400;      //[ms]

void setup() {
  // For the USB, just use Serial as normal:
  Serial.begin(115200);

  // Configure MySerial0 on pins RX=D7 and TX=D6 (-1, -1 means use the default)
  MySerial0.begin(4800, SERIAL_8N1, -1, -1);


  Reset();
  setFrequency(60);    //50[Hz]
  setUpdateRate(800);  //400[ms]
}

void loop() {
  float voltage;
  getVoltage(&voltage);
  Serial.printf("%.2f [V]\n", voltage);

  float current;
  getCurrent(&current);
  Serial.printf("%.2f [A]\n", current);

  float activePower;
  getActivePower(&activePower);
  Serial.printf("%.2f [W]\n", activePower);

  float activeEnergy;
  getActiveEnergy(&activeEnergy);
  Serial.printf("%.3f [kWh]\n", activeEnergy);

  float powerFactor;
  getPowerFactor(&powerFactor);
  Serial.printf("%.1f [%%]\n", powerFactor);

  float temperature;
  getTemperature(&temperature);
  Serial.printf("%.1f [deg C]\n", temperature);

  Serial.println("");
  delay(1000);
}

uint8_t _culcCheckSum(uint8_t *txData, int txLenght, uint8_t *rxData, int rxLenght) {

  uint8_t checksum = 0;
  for (int i = 0; i < txLenght; i++) {
    checksum += txData[i];
  }
  for (int i = 0; i < rxLenght; i++) {
    checksum += rxData[i];
  }
  checksum = ~checksum;
  return checksum;
}

bool _writeRegister(uint8_t address, uint32_t data) {
  //read buffer clear
  while (MySerial0.available() != 0) {
    MySerial0.read();
  }

  //Register Unlock
  uint8_t unlockTxData[6] = { 0xA8, 0x1A, 0x55, 0, 0, 0 };
  unlockTxData[5] = _culcCheckSum(unlockTxData, sizeof(unlockTxData) - 1, 0, 0);
  MySerial0.write(unlockTxData, sizeof(unlockTxData));

  //Write Register
  uint8_t txData[6] = { 0xA8, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16) };
  txData[5] = _culcCheckSum(txData, sizeof(txData) - 1, 0, 0);
  MySerial0.write(txData, sizeof(txData));

  return true;
}

bool _readRegister(uint8_t address, uint32_t *data) {
  uint8_t txData[] = { 0x58, address };
  MySerial0.write(txData, sizeof(txData));

  uint8_t rxData[4] = { 0, 0, 0, 0 };
  uint32_t startTime = millis();
  while (MySerial0.available() != sizeof(rxData)) {
    delay(10);
    if ((millis() - startTime) > timeout)
      break;
  }
  int rxDataLength = MySerial0.readBytes(rxData, sizeof(rxData));

  if (rxDataLength == 0) {
    Serial.println("Serial Timeout.");
    return false;
  }

  uint8_t checksum = _culcCheckSum(txData, sizeof(txData), rxData, sizeof(rxData) - 1);
  if (rxData[3] != checksum) {
    char massage[128];
    sprintf(massage, "Checksum error truet:%x read:%x.", checksum, rxData[3]);
    Serial.println(massage);
    return false;
  }

  *data = ((uint32_t)rxData[2] << 16) | ((uint32_t)rxData[1] << 8) | (uint32_t)rxData[0];
  return true;
}

bool getCurrent(float *current) {
  uint32_t data;
  if (false == _readRegister(0x04, &data)) {
    Serial.println("Can not read I_RMS register.");
    return false;
  }
  *current = (float)data * Vref / ((324004.0 * R5 * 1000.0) / Rt);
  return true;
}

bool getVoltage(float *voltage) {
  uint32_t data;
  if (false == _readRegister(0x06, &data)) {
    Serial.println("Can not read V_RMS register.");
    return false;
  }

  *voltage = (float)data * Vref * (R8 + R9 + R10 + R11 + R12) / (79931.0 * R7);
  return true;
}

bool getActivePower(float *activePower) {
  uint32_t data;
  if (false == _readRegister(0x08, &data)) {
    Serial.println("Can not read WATT register.");
    return false;
  }

  int32_t rowActivePower = (int32_t)(data << 8) / 256;
  if (rowActivePower < 0)
    rowActivePower = -rowActivePower;
  *activePower = (float)rowActivePower * Vref * Vref * (R8 + R9 + R10 + R11 + R12) / (4046.0 * (R5 * 1000.0 / Rt) * R7);
  return true;
}

bool getActiveEnergy(float *activeEnergy) {

  uint32_t data;
  if (false == _readRegister(0x0A, &data)) {
    Serial.println("Can not read CF_CNT register.");
    return false;
  }

  int32_t rowCF_CNT = (int32_t)(data << 8) / 256;
  if (rowCF_CNT < 0)
    rowCF_CNT = -rowCF_CNT;
  //Serial.print("Float de Energia: ");
  //Serial.println(rowCF_CNT);
  *activeEnergy = (float)rowCF_CNT * 1638.4 * 256.0 * Vref * Vref * (R8 + R9 + R10 + R11 + R12) / (3600000.0 * 4046.0 * (R5 * 1000.0 / Rt) * R7);

  return true;
}

bool getPowerFactor(float *powerFactor) {
  uint32_t data;
  if (false == _readRegister(0x0C, &data)) {
    Serial.println("Can not read CORNER register.");
    return false;
  }

  float rowPowerFactor = cos(2.0 * 3.1415926535 * (float)data * (float)Hz / 1000000.0) * 100.0;
  if (rowPowerFactor < 0)
    rowPowerFactor = -rowPowerFactor;
  *powerFactor = rowPowerFactor;

  return true;
}

bool getTemperature(float *temperature) {
  uint32_t data;
  if (false == _readRegister(0x0E, &data)) {
    Serial.println("Can not read TPS1 register.");
    return false;
  }

  int16_t rowTemperature = (int16_t)(data << 6) / 64;
  *temperature = (170.0 / 448.0) * (rowTemperature / 2.0 - 32.0) - 45;
  return true;
}

bool Reset() {
  if (false == _writeRegister(0x19, 0x5A5A5A)) {
    Serial.println("Can not write SOFT_RESET register.");
    return false;
  }
  while (MySerial0.available() != 0) {
    MySerial0.read();
  }

  delay(500);
  return true;
}

bool setUpdateRate(uint32_t rate) {
  uint32_t data;
  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }

  uint16_t mask = 0b0000000100000000;  //8bit
  if (rate == 400)
    data &= ~mask;
  else
    data |= mask;

  if (false == _writeRegister(0x18, data)) {
    Serial.println("Can not write MODE register.");
    return false;
  }

  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }

  if ((data & mask) == 0) {
    updateRate = 400;
    Serial.println("Set update rate:400ms.");
  } else {
    updateRate = 800;
    Serial.println("Set update rate:800ms.");
  }
  return true;
}

bool setFrequency(uint32_t Hz) {
  uint32_t data;
  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }

  uint16_t mask = 0b0000001000000000;  //9bit
  if (Hz == 50)
    data &= ~mask;
  else
    data |= mask;

  if (false == _writeRegister(0x18, data)) {
    Serial.println("Can not write MODE register.");
    return false;
  }

  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }

  if ((data & mask) == 0) {
    Hz = 50;
    Serial.println("Set frequency:50Hz");
  } else {
    Hz = 60;
    Serial.println("Set frequency:60Hz");
  }
  return true;
}

bool setOverCurrentDetection(float detectionCurrent) {
  const float magicNumber = 0.72;  // I_FAST_RMS = 0.72 * I_RMS (Values obtained by experiments in the case of resistance load)

  //MODE[12] CF_UNABLE set 1 : alarm, enable by TPS_CTRL[14] configured
  uint32_t data;
  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }
  data |= 0b0001000000000000;  //12bit
  if (false == _writeRegister(0x18, data)) {
    Serial.println("Can not read write register.");
    return false;
  }

  //TPS_CTRL[14] Alarm switch set 1 : Over-current and leakage alarm on
  if (false == _readRegister(0x1B, &data)) {
    Serial.println("Can not read TPS_CTRL register.");
    return false;
  }
  data |= 0b0100000000000000;  //14bit  0b0100000000000000
  if (false == _writeRegister(0x1B, data)) {
    Serial.println("Can not write TPS_CTRL register.");
    return false;
  }

  //Set detectionCurrent I_FAST_RMS_CTRL
  data = (uint32_t)(detectionCurrent * magicNumber / Vref * ((324004.0 * R5 * 1000.0) / Rt));
  data >>= 9;
  data &= 0x007FFF;
  float actualDetectionCurrent = (float)(data << 9) * Vref / ((324004.0 * R5 * 1000.0) / Rt);
  data |= 0b1000000000000000;  //15bit=1 Fast RMS refresh time is every cycle
  data &= 0x00000000FFFFFFFF;
  if (false == _writeRegister(0x10, data)) {
    Serial.println("Can not write I_FAST_RMS_CTRL register.");
    return false;
  }
  char massage[128];
  sprintf(massage, "Set Current Detection:%.1fA.", actualDetectionCurrent);
  Serial.println(massage);

  return true;
}

bool setCFOutputMode() {
  //MODE[12] CF_UNABLE set 0 : alarm, enable by TPS_CTRL[14] configured
  uint32_t data;
  if (false == _readRegister(0x18, &data)) {
    Serial.println("Can not read MODE register.");
    return false;
  }
  data &= ~0b0001000000000000;  //12bit
  if (false == _writeRegister(0x18, data)) {
    Serial.println("Can not read write register.");
    return false;
  }
}