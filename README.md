# Welcome to Fursuit32! 🐾
this is a hobby project of mine that provides an 'easy' confgurable ESP32 interface.
Fursuit32 uses Bluetooth Low Energy (BLE), allowing you to configure and customize how you interact with for example lights and/or fans in your fursuit.

# Requirements
- ESP32
- Arduino IDE 2.3.6+
- A BLE-compatible browser (recommend Bluefy)
- Some knowledge of c++, HTML and aurdino IDE would come in handy

# Setup
1. Install Arduino IDE 2.3.6 or newer.
2. Open the Arduino IDE and install the ESP32 boards through the Board Manager.
3. Connect your ESP32 and select the correct board and USB/COM port.
4. Flash the Fursuit32 firmware to your ESP32.
5. Open the Fursuit32 web interface in a browser that supports BLE. On iOS, Bluefy is one supported option.
6. Connect to your Fursuit32:
6.1. On first boot it might take a few trys. the closer the better. when connected it will ask you to configure your pins to sliders. you can wire an LEDS to GPIO 25 and 26. and combine the 2 pins for an LED slider. you can put them in the eyes or paws or where ever you like :3
6.2. You can customize the looks of the thing by pressing
7. Customize it to your needs and have fun!

# Setup NFC with Bluefy and Fursuit32
Write this: bluefy://open?url=https://kaasijs.github.io/Fursuit32/ URL to a NFC hide it in your suit and your done!
