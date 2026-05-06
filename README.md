# STM32F405 Stepper controller with DRV8424
This is the zephry Firmware to controll the DRV8424 to drive a stepper.

## Usage
The current code enables to start a predefined movement 2 rotations in one direction and 2 rotations in the other direction. This movement is used to get a measurement of a motorpositionsensor (MPS). To start the routine the Controller has to be connected via the USB-c connector. To connect to the device run the following command on the pc
```
minicom -D /dev/ttyACM0 -b 115200
```
After starting the device by connecting it to a 12V/1A power supply. The device will start in to the interactive shell. The routine can now be triggered by entering this command
```
start_process
```
Alternativly by running `help` all available commands will be listed.

