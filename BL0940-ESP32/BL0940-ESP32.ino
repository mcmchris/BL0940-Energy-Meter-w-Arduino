
#include "BL0940.h"

/* Define the board used (Uncomment it)*/


BL0940 bl0940;

void setup() {
  Serial.begin(115200);

  bl0940.Reset();
  bl0940.setFrequency(60); //50[Hz]
  bl0940.setUpdateRate(800); //400[ms]
}

void loop() {

  float voltage;
  bl0940.getVoltage( &voltage );
  Serial.printf("%.2f [V]\n", voltage );

  float current;
  bl0940.getCurrent( &current );
  Serial.printf("%.2f [A]\n", current );

  float activePower;
  bl0940.getActivePower( &activePower );
  Serial.printf("%.2f [W]\n", activePower );

  float activeEnergy;
  bl0940.getActiveEnergy( &activeEnergy );
  Serial.printf("%.3f [kWh]\n", activeEnergy );

  float powerFactor;
  bl0940.getPowerFactor( &powerFactor );
  Serial.printf("%.1f [%%]\n", powerFactor );

  float temperature;
  bl0940.getTemperature( &temperature );
  Serial.printf("%.1f [deg C]\n", temperature );

  Serial.println("");
  delay(1000);

}
