# Electronics Components

## Controller and Sensors

- **ESP32-S3-DevKitC-1-N8R8 (x1):** Main microcontroller that runs everything.
- **SparkFun BMI270 IMU breakout (x1):** Motion and orientation sensing.
- **MS5837-30BA depth sensor (x1):** Measures depth and water pressure.
- **AD8232 ECG module (x1):** Reads heart signals.
- **Dry ECG electrodes, RA / LA / RL (x1 set):** Skin contacts for the ECG.

## Power

- **LiPo battery, 3.7 V 2000 mAh 2C (x1):** Main power source.
- **TP4056 charger module (x1):** Charges and protects the battery.
- **Pololu S8V9F3 regulator (x1):** Provides steady 3.3 V for the ESP32 and sensors.
- **XL6009 boost module (x1):** Steps battery voltage up to 5 V for the servo.
- **470 µF 16 V electrolytic capacitor (x1):** Absorbs the servo’s startup current so the ESP32 doesn’t reset.

## Actuation and Alerts

- **Servo motor (x1):** Opens the valve that inflates the bladder.
- **3-pin servo header (x1):** Connects the servo (signal to GPIO 16, +5 V, GND).
- **AO3400A N-MOSFET, SOT-23 (x2):** Switches the vibration motor and the strobe.
- **100 Ω resistor (x2):** Limits current into the MOSFET gates.
- **100 kΩ resistor (x2):** Keeps the MOSFETs off during startup.
- **SS14 Schottky diode (x1):** Protects against voltage spikes when the motor shuts off.
- **Coin vibration motor (x1):** Vibration alert for the diver.
- **Evan Designs strobe (x1):** Visual distress light.

## Controls

- **Pushbutton (x2):** Cancel and SOS buttons.