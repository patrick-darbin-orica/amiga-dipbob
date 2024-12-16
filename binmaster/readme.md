# BinMaster Firmware 

Firmware for running the modified SmartBob unit on a Nucleo L432KC

## Build
### System Requirements
* arm-none-eabi-gcc (may also be called gcc-arm-none-eabi)
* cmake (If not using bundled version with Clion)
* openocd (May need to install OpenOCD udev rules to /etc/udev/rules.d/ and add user to group plugdev depending on distro)
* STM32CubeMX
* STLink Driver (may be required on windows)

#### Highly Recommended for Development
* CLion with OpenOCD + STM32CubeMX plugin (Latest versions may not require plugin, now built in functionality)

CLion is not technically required as it should be possible to build just using cmake, however development will be a bit
more of a challenge without a decent IDE. 

For full instructions on installing the addon with CLion see 
[installation guide v2](https://blog.jetbrains.com/clion/2017/12/clion-for-embedded-development-part-ii/) for pre 2019.1 EAP
releases or 
[installation guide v3](https://blog.jetbrains.com/clion/2019/02/clion-2019-1-eap-clion-for-embedded-development-part-iii/)
for post 2019.1 EAP releases

### Setup Project
General steps for building the project are as follows. 

* Git clone repository
* Select board/st_nucleo_l4.cfg or board/st_nucleo_l476rg.cfg in either Run -> Edit Configurations -> OCD binmaster
 -> Board Config File 
* Open binmaster.ioc in STM32CubeMX and Generate Code (The little gear symbol)
* **For pre 2019.1 CLion:** select the menu item "Tools -> Update CMake project with STM32CubeMX project". Note this only needs to be 
selected manually once. After the first time it'll automatically pick up any further STM32CubeMX generates. 

The first steps can be done from command line on linux or git bash in windows as follows. 
```bash 
git clone https://<user>@bitbucket.org/orica/surface-automation.git
```

## Programing
### With CLion
If all setup steps as above have been done the following should program a board.

* Connect STM32 usb programmer to board and pc
* Select 'OCD binmaster | Debug' within run configurations (top right).
* Select play button (shift + f10)
* Prompt at bottom of the screen should announce firmware download success

For debugging

* Insert breakpoints at desired location be selecting directly to the right of the line number on the right hand side
of the screen. 
* As per above programming instruction, however select the bug button rather than play (shift + f9)
* Debugger console should automatically open

### With CMake
```bash
mkdir build
cd build
cmake..
make
/usr/bin/openocd -c "tcl_port disabled" -s /usr/share/openocd/scripts -f board/st_nucleo_l476rg.cfg \
 -c "program \"[LOCATION TO REPO]/binmaster/build/binmaster.elf\"" -c reset -c shutdown
```