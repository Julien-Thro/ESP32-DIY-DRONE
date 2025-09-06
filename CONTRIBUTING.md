# Contribuer

Merci d’aider ce projet 👋

## Pré-requis
- Arduino IDE **ou** `arduino-cli`
- Core **ESP32** installé
- Libs: `RF24`, `MPU6050_light` (Library Manager), `DShotRMT` incluse dans `lib/`

## Branches
- `main` : stable
- `dev`  : intégration (PR vers `dev`)

## Style
- C++11/14 Arduino.
- Indent 2 espaces, LF, UTF-8.
- Noms explicites, pas d’abréviations obscures.
- Commits **Conventional Commits**:
  - `feat:`, `fix:`, `docs:`, `refactor:`, `perf:`, `test:`, `build:`, `ci:`, `chore:`

## Tests locaux
- Compiler avec Arduino IDE **ou**:
  ```bash
  arduino-cli core install esp32:esp32
  arduino-cli lib install "RF24" "MPU6050_light"
  arduino-cli compile --fqbn esp32:esp32:esp32 firmware
