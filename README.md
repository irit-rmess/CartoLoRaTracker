# CartoLoRaTracker

## Platformio

CartoLoRaTracker recommends Platformio.

1. Follow https://docs.platformio.org/en/latest/core/installation.html#installation-methods to install Platformio
2. Clone the Radiohead library https://lab.iut-blagnac.fr/adrien/radiohead-lab in `lib/` directory
3. Clone the Locapack library https://lab.iut-blagnac.fr/adrien/locapack in another directory outside the project
4. Make a symbolic link of the `arduino/locapack` directory in the `lib/` directory
5. Connect the M5-stack with LoRa and GPS shield (optionnal) to a USB port
6. `pio run -t upload` to compile and flash

## WiFi
In CartoLoRaTracker.ino:
* Set SSID
* Set password

## GPS

2 options are available

### Over serial
Add GPS shield

In CartoLoRaTracker.ino:
* Use GNSS_SERIAL to define the corresponding serial

### Over UDP
In CartoLoRaTracker.ino:
* Define GNSS_UDP
* Define listing port with GNSS_UDP_PORT

Android GPS forwarding
* [GPSd Forwarder](https://f-droid.org/packages/io.github.tiagoshibata.gpsdclient/)
