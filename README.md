# profikum
A profinet interface for the profikum X3 zero mobile robot.

## Licence
profikum itself is licenced under the GPL version 3. See LICENSE for details. 

## Installation
Needed: 
- Raspberry Pi 3 Model B+ or higher, or Banana Pi M2 Zero or higher
- Pololu Zumo 32U4. The Arduino version should also work, but the 32U4 is assumed in the following.
Steps:
- Install profi++
- Install the Arduino IDE on the Raspberry: https://www.arduino.cc
- In the Arduino IDE, install the board driver of the 32U4:
  - Open File->Preferences. Enter https://files.pololu.com/arduino/package_pololu_index.json in the "Additional Board Manager URLs" text box. Press OK.
  - In Tools->Boards->Boards Manager, search and install "Pololu A-Star Boards"
  - In Tools->Board, select "Pololu A-Star 32U4"
- In the Arduino IDE, install the library of the 32U4:
  - Select Sketch->Include library->Manage Libraries
  - Search and "Zumo32U4"
  - If you are asked to install missing dependencies: install them all!
- In the Arduino IDE, upload the Arduino program (~/profipp/examples/profizumo/resources/profizumo/
- Configure to connect to iWLAN on startup:
  - In the terminal, type ``sudo nano /etc/network/interfaces``. Add/modify the following lines:
    ``` 
	allow-hotplug wlan0
	iface wlan0 inet static
	address 192.168.0.40 # or whatever IP the Raspberry should have
	netmask 255.255.255.0 # Or whatever subnet mask it should have
	gateway 192.168.1.1 # or whatever is your default gateway
	wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf 
	```
  - In the terminal, type ``sudo nano /etc/wpa_supplicant/wpa_supplicant.conf``. Add/modify the following lines:
    ```
	network={
         ssid="F5.29_wlan" # your wifi's name
         psk="password123" # your wifi's password
         key_mgmt=WPA-PSK
    }
	```
- Compile profikum
- Generate the GSDML file:
  - Run ``./profikum -e ./``
  - Copy the GSDML (e.g. via a USB stick) to the computer of the PLC.
  - Start TIA Portal, install the GSDML and add it to the installed devices.
- Raspberry Pi OS (Raspberry Pi 3 Model B+ or higher): Configure serial port/UART0:
  - Type ``sudo nano /boot/config.txt``.
  - Find commented out line ``#dtparam=spi=on`` and remove comment, i.e. such that it reads ``#dtparam=spi=on``.
  - Add the following lines at the bottom of config.txt:
    ```
	enable_uart=1
	dtoverlay=uart0
	```
  - There might have been other steps which have to be done, depending on the configuration. See e.g.: 
    - https://maker-tutorials.com/uart-schnittstelle-am-raspberry-pi-aktiveren/
  - To test serial communication, the easiest is to first directly connect TX and RX of the Raspberry. It will thus send information to itself, such that you definitely know if something is not working due to the Raspberry config, or due to something else.
- Armbian (Banana Pi M2 Zero or higher):
  - Enable UART 3 in Settings->Armbian Settings to enable UART via pins 8 and 10. Reboot.
  - In the following, use /dev/ttyS3 .
- Configure to autostart profikum on startup
  - Type ``sudo nano /lib/systemd/system/profikum.service`` into the console.
  - Add the following content to the file:
    ```
	[Unit]
	Description=profikum launcher
	
	[Service]
	Type=idle
  WorkingDirectory=/home/username/profikum/build
	# Wait 10s upon reboot to ensure wifi is up and running
	ExecStartPre=/bin/sleep 10
	# replace path to whereever you installed profizumo
	ExecStart=/usr/bin/sudo /home/username/profikum/build/profikum -s -e ./ -i wlan0 -u /dev/ttyS3
	
	[Install]
	WantedBy=default.target
	```
  - Change access permissions: ``sudo chmod 644 /lib/systemd/system/profikum.service``
  - Tell systemd to run this service:
    ```
    sudo systemctl daemon-reload
    sudo systemctl enable profikum
	```
  - Test the service: ``sudo systemctl start profikum``. Then, wait more than 10s and check the status: ``sudo systemctl status profikum``. 
  - If everything works fine: reboot (``sudo reboot``), wait a few seconds and test status again: ``sudo systemctl status profikum``
  