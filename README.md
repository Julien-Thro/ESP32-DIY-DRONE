# ESP32 FPV Quad – NRF24 + DShotRMT + MPU6050 (Acro)

Firmware monolithique `.ino` pour quad X basé sur **ESP32** (Arduino), **NRF24L01**, **ESC DSHOT** via **DShotRMT**, et **MPU6050** en mode **Acro (rate)**.

## Caractéristiques
- Radio **NRF24** (manette ESP32), 4 canaux (throttle, pitch, roll, yaw).
- ESC en **DSHOT1200** (API 2025 : `begin()`, `sendThrottle()`, `sendCommand()`).
- **MPU6050** + **PID rate** (roll/pitch). Yaw direct (option PID à venir).
- **Arming**: throttle < 50 + **soft-start** anti-surge.
- **Failsafe**: STOP moteurs si perte radio.
- **Batterie**: lecture VBAT (pont 68k/10k sur GPIO34), buzzer pin 13 (désactivé si Vbat < 10 V).
- Logs série: Vbat, gyro rates, PID out, états arm/failsafe.

## Matériel (référence)
- **ESP32 Dev Module**
- **NRF24L01** (CE=GPIO4, CSN=GPIO5, canal 110)
- **MPU6050** (I2C)
- **ESC DSHOT** (quad X)
- **Pont diviseur** VBAT: R1=68 kΩ / R2=10 kΩ → GPIO34
- **Buzzer**: YX5020P sur GPIO13

## Pinout (quad X)
| Fonction  | GPIO | Note                       |
|-----------|------|----------------------------|
| Moteur 1  | 16   | FL (CCW)                   |
| Moteur 2  | 17   | RL (CW)                    |
| Moteur 3  | 32   | RR (CCW)                   |
| Moteur 4  | 33   | FR (CW)                    |
| Buzzer    | 13   | PWM tone()                 |
| VBAT      | 34   | ADC 12 bits (pont 68k/10k) |
| NRF24 CE  | 4    |                            |
| NRF24 CSN | 5    |                            |
| I2C SDA   | 21   | MPU6050                    |
| I2C SCL   | 22   | MPU6050                    |

## Dépendances Arduino
- **ESP32 core** (Boards Manager)  
- **RF24** (TMRh20)  
- **MPU6050_light**  
- **DShotRMT** (version 2025, incluse dans `lib/DShotRMT/` du repo)

## Build rapide (Arduino IDE)
1. Installer **ESP32** via Board Manager, sélectionner **ESP32 Dev Module**.
2. Installer les libs via Library Manager: `RF24`, `MPU6050_light`.  
   **DShotRMT** est fournie dans `lib/`.
3. Ouvrir `firmware/esp32_fpv_quad.ino`, choisir le bon port, **Upload**.

## Build CI local (arduino-cli)
```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "RF24" "MPU6050_light"
arduino-cli compile --fqbn esp32:esp32:esp32 firmware
