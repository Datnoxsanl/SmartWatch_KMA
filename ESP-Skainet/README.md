# MK39 Offline Voice Command Control System
![MK39](assets/cover.jpg)
## Overview

For video instructions on using this repository, kindly check out my YouTube channel where I will post tutorials and other information on various project.

[YouTube Channel](https://www.youtube.com/@KishSan) \
[Link to Instructional Video](https://www.youtube.com/watch?v=_AuPDKQoaPo)

This repository contains code build off the ESP-SKAINET framework which was designed specifically to be used on the following boards:
 * ESP32-S3-Korvo-1
 * ESP-BOX
 * ESP-S3-Korvo-2
 * ESP32-S3-EYE
 * ESP32-P4-Function-EV

The purpose of this repo is to start with a simplified speech recognition framework that will be easy to update and add future functionality. The features and concepts included here are meant to be used across various robotics projects.

The GitHub repository for the official framework can be found here:
* [ESP-Skainet Framework](https://github.com/espressif/esp-skainet)

Modifications were needed to the framework to work with DevKit modules. Special thanks to Eric for this repository that enables us to work with the DevKit modules:
* [ESP-Skainet framework modification for ESP32-S3 DevKitC](https://github.com/0015/esp-skainet/tree/ESP32-S3-Devkit-C)

Due to the nature of the project, the framework was stripped down to only contain necessary modules for this application. Features including:
* English speech recognition
* Servo control using LEDC (8 channels)
    * MCPWM implementation has been removed but can be used for additional channels if needed.
* Addressable LED control using RMT drivers

The final revision of this code base was tested on both the ESP32-S3 N8R8 and ESP32-S3 N16R8 modules. Different variations will need to be configured in the SDK using the menuconfig option.

## Included Demo Features
The system will boot, awaiting an activation command. This repo has been pre-configured for 'JARVIS' as the activation command. Multiple activation commands can be set as well to use various wake words for system control.

![Activation](assets/start.png)

If no command is issued within a certain time, a timeout will occur and return to a waiting state. At this point an activation command can be issued again.

![Timeout](assets/timeout.png)

Example sequences are displayed for Opening and Closing the helmet. Phrases are matched based on various probabilities so similar words may cause conflicts.

![Open](assets/open.png)

![Close](assets/close.png)

## Additional Hardware Required

* INMP 441 MEMS Microphone
    * This addition is required for the speech input when using the DevKit modules

## Default GPIO Configuration
```
Helmet Servo 1      - GPIO 4
Helmet Servo 2      - GPIO 5
Eye LED             - GPIO 38
Addressable LEDs    - GPIO 21
```

### Configuring the GPIOs for the INMP Module
* Navigate to the folder:
    * mk39-speech-recognition\components\hardware_driver\boards\include\'
    * Edit the file 'esp32_s3_devkit_c.h'
* Edit the following fields depending on the configuration of your INMP module. Refer to the pinout diagram for a proper port selection for these inputs if you choose a different layout.

```
/**
 * @brief ESP32-S3-DEVKIT-C I2S GPIO defination
 * 
 */
#define FUNC_I2S_EN         (1)
#define GPIO_I2S_LRCK       (GPIO_NUM_11)
#define GPIO_I2S_MCLK       (GPIO_NUM_NC)
#define GPIO_I2S_SCLK       (GPIO_NUM_12)
#define GPIO_I2S_SDIN       (GPIO_NUM_10)
#define GPIO_I2S_DOUT       (GPIO_NUM_NC)
```

## Setting up the ESP-IDF environment
Development for this project was done with the ESP-IDF extension in VSCode. It was developed to support up to ESP-IDF v5.4. There is also a side branch with older v5.0.8 support.

Further details on setting up the IDF environment can be found here and is outside the scope of this documentation.
* [ESP-IDF Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)

Once the environment has been configured and setup properly you can test by opening an ESP-IDF terminal. This can be accessed from the extension on the left in VSCode, which will open a terminal and run the export script to set environment variables among other setting needed for development.

## Configure, Build and Flash

Once the ESP-IDF development environment has been set up, we can begin configuring and flashing the project.

Before you begin, ensure that you are in the project root:
```
'mk39-speech-recognition\' 
or your specific project folder name
```

##### set-target 

This is important to compile the proper binaries for the device you are using. Since the sdkconfig file included in this repo is specific for the esp32s3, it should work out of the box.
```
idf.py set-target esp32s3
```

##### configure

The sdkconfig file contains the following command activations and phrases:

* Activation: Jarvis (wn9_jarvis_tts)
* Commmands (not case-sensitive):
    * Wake Up (WdK cP)
    * Hulk Out (hcLK tT)
    * Let's Go (LfTS Gb)

Manual SDK changes can be done through menuconfig:
```
idf.py menuconfig
```

#### Configuring Command Words
To generate phonemes for your input phrases, navigate to the folder:
* "mk39-speech-recognition\managed_components\espressif__esp-sr\tool"
* Execute the command:
```
    python .\multinet_g2p.py -t "Custom Phrase"
```
* You will need python and necessary dependencies installed to run the script. Refer to the ESP-SR documentation on setting this up.

To reset the framework configurations to the default configuration for the ESP32-S3 module, use the following command example to copy the pre-existing defaults to the 'sdkconfig' file.

```
cp sdkconfig.defaults.esp32s3 sdkconfig
```

##### build&flash

Build the project and flash it to the board, then run the monitor tool to view the output via serial port:

```
idf.py build

idf.py flash monitor 
```

To enter monitor only mode and also reset the ESP32-S3 you can use the following:
```
idf.py monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Modify speech commands

We recommend using MultiNet6 or newer models.   
Here's a simple example to modify speech commands in the code.  
You can also modify the default command list, please refer to [document](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/speech_command_recognition/README.html) for more details.

```
// MultiNet6
// Note: Please create multinet handle before adding speech commands

esp_mn_commands_clear();                       // Clear commands that already exist 
esp_mn_commands_add(1, "turn on the light");   // add a command
esp_mn_commands_add(2, "turn off the light");  // add a command
esp_mn_commands_update();                      // update commands
multinet->print_active_speech_commands(model_data);     // print active commands
```


## Modifying Wake Word

There will be changes done to the ESP-SR framework which is what skainet uses for the wake words and command recognition. Updating to the latest framework version can provide additional wake words and features when needed.

[ESP-SR Framework](https://github.com/espressif/esp-sr)

For this project, the ESP-SR has been included as a managed component. To add future functionality to the framework, the version can be modified in the following file:
```
main\idf_component.yml
```
The current version of ESP-SR in this build is v2.0.0 which brings several improvements to the AFE and adds additional features such as noise supression.

Note that the v2 AFE interface is not compatible with ESP-SR versions 1.x.x. The migration procedure has been implemented in this build so you will only be able to use v2.x.x

You can view the changelog here to stay up to date on the latest features.

[ESP-SR Changelog](https://github.com/espressif/esp-sr/blob/master/CHANGELOG.md)

## Future Functional Improvements

Future project scope include:
* Voice playback functionality (TTS)