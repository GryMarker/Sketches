**Getting Started**
To get these projects up and running, you'll need to follow a few simple steps.

Install the Arduino IDE: Download and install the official Arduino Integrated Development Environment (IDE) from the Arduino website.

Add ESP32 Board Manager: The Arduino IDE doesn't support the ESP32 out of the box. You need to manually add the ESP32 board manager URL in the preferences.

In the Arduino IDE, go to File > Preferences.

In the "Additional Boards Manager URLs" field, paste this URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json.

Install ESP32 Boards:
Go to Tools > Board > Boards Manager....
Search for "esp32" and install the "esp32 by Espressif Systems" package.

Connect and Upload:
Connect your ESP32 board to your computer with a USB cable.
Select your board from Tools > Board (e.g., "DOIT ESP32 DEVKIT V1").
Select the correct COM port under Tools > Port.
Open the desired project sketch from this repository in the Arduino IDE.
Click the "Upload" button (the right-arrow icon) to send the code to your ESP32.

The projects in this repository are provided as educational resources. While we've done our best to make them reliable, they are intended to be a starting point, 
not as a fully finished product. 
The students are encouraged to experiment and modify the code and hardware as they see fit. 
Any changes or modifications to the projects are the full responsibility of the user.

If you have any questions, suggestions for improvements, 
or have a cool addition you'd like to share, please don't hesitate to contact us again or create an issue in this repository.
