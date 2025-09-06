# Build & Flash

## Arduino IDE
1. Installer **ESP32** via File → Preferences → Additional Boards URL, puis Boards Manager.
2. Sélectionner **ESP32 Dev Module**.
3. Installer libs `RF24`, `MPU6050_light` (Library Manager).
4. Le dossier `lib/DShotRMT` est inclus localement.
5. Ouvrir `firmware/esp32_fpv_quad.ino` et **Upload**.

## arduino-cli
```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "RF24" "MPU6050_light"
arduino-cli compile --fqbn esp32:esp32:esp32 firmware
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware

---

### 14) `docs/WIRING.md`
```markdown
# Câblage & Schémas

## NRF24L01
- CE → GPIO4
- CSN → GPIO5
- VCC 3.3 V (avec condo 10 µF proche module recommandé)
- SPI par défaut (ESP32): SCK=18, MISO=19, MOSI=23

## MPU6050
- SDA → GPIO21
- SCL → GPIO22
- VCC 3.3 V / 5 V selon module (GND commun)

## ESC (DShot)
- M1 (FL CCW) → GPIO16
- M2 (RL CW)  → GPIO17
- M3 (RR CCW) → GPIO32
- M4 (FR CW)  → GPIO33

## Buzzer
- Buzzer + → GPIO13
- Buzzer − → GND

## VBAT
- Batterie + → R1=68k → ADC GPIO34
- ADC GPIO34 → R2=10k → GND
- Calibrer `VBAT_RATIO` si besoin (multimètre).
