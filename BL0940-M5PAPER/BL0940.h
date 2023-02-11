/* BL0940 Calibration-free Metering IC control      */
/*                      2022.Sep.23 kohacraft.com   */
/*                           Creative Commons CC0   */
/* https://kohacraft.com/archives/202209020915.html */

#ifndef BL0940_h
#define BL0940_h

class BL0940
{
  public:
    BL0940();
    ~BL0940();
    
    bool getCurrent( float *current );  //[A]
    bool getVoltage( float *voltage );  //[V]
    bool getActivePower( float *activePower );  //[W]
    bool getActiveEnergy( float *activeEnergy );  //[kWh]
    bool getPowerFactor( float *powerFactor );  //[%]
    bool getTemperature( float *temperature );  //[deg C]
    bool setFrequency( uint32_t Hz = 60 );  //50 or 60  [Hz]
    bool setUpdateRate( uint32_t rate = 400 );  //400 or 800  [ms]
    bool setOverCurrentDetection( float detectionCurrent = 15.0 );  //[A] CF pin is high if current is larger than detectionCurrent
    bool setCFOutputMode(); //Enegy pulse output CF pin
    bool Reset();

  private:
    const uint16_t timeout = 1000;  //Serial timeout[ms]
    const float Vref = 1.218; //[V]
    const float R5 = 3.3;   //[Ohm]
    const float Rt = 2000.0;  //n:1
    const float R8 = 20.0;  //[kOhm]
    const float R9 = 20.0;  //[kOhm]
    const float R10 = 20.0;  //[kOhm]
    const float R11 = 20.0;  //[kOhm]
    const float R12 = 20.0;  //[kOhm]
    const float R7 = 24.9;  //[Ohm]
    uint16_t Hz = 60;   //[Hz]
    uint16_t updateRate = 400; //[ms]

    uint8_t _culcCheckSum( uint8_t *txData , int txLenght , uint8_t *rxData , int rxLenght );
    bool _writeRegister( uint8_t address , uint32_t data );
    bool _readRegister( uint8_t address , uint32_t *data );
};
#endif /* BL0940 */
