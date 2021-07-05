# CartoLoRaTracker

## Platformio

CartoLoRaTracker recommands Platformio.

1. Follow https://docs.platformio.org/en/latest/core/installation.html#installation-methods to install Platformio
2. Clone the Radiohead library https://lab.iut-blagnac.fr/adrien/radiohead-lab in `lib/` directory
3. Clone the Locapack library https://lab.iut-blagnac.fr/adrien/locapack in another directory outside the project
4. Make a symbolic link of the `arduino/locapack` directory in the `lib/` directory
5. Connect the M5-stack with LoRa and GPS shield to a USB port
6. `pio run -t upload` to compile and flash
