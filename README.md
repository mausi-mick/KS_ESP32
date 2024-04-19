
   Kennlinienschreiber mit ESP32

   


Hier möchte ich nur kurz die ESP32-Variante vorstellen.

Die vorher beschriebenen Versionen  -  auf Arduino NANO / ATMega328P   bzw. Arduino-EVERY basierend   - waren im Laufe der Entwicklung in mehrerer Hinsicht an ihre Grenzen gestossen:

    •     zu wenig Speicher
    •     zu wenig Pins
    •     relativ langsam
    •     Bluetooth nur über externe HW (HC05 /HC12)



Die ersten Tests waren dann aber etwas ernüchternd,  

Gerade die verwandten ADC’s (MCP3204) und DAC’s MCP4822 an SPI (VSPI) machten
Probleme., aber das scheint gelöst zu sein.

Ich verwende ein ESP-Modul mit 38 Pins, um möglichst viele Pins zur Verfügung zu haben für
die BUSSE (I2C, SPI), Schalter und Encoder und  RX/TX (z.B. zur Verbindung mit modif. Transistortester)
 
