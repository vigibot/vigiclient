# Make your own Vigibot.com raspberry PI robot

## Installation on a clean Raspbian Stretch Lite

### Prerequisites

- Flash the last Raspbian Stretch Lite image: https://www.vigibot.com/raspbian
- Put your "wpa_supplicant.conf" and an empty "ssh" file inside the boot partition
- Connect to your Raspberry Pi via SSH
- sudo apt update
- sudo apt upgrade
- sudo raspi-config
- Enable camera and I2C

### Installation

- wget https://www.vigibot.com/vigiclient/install.sh
- sudo bash install.sh
- sudo nano /boot/robot.json
- Change the "Demo" login and the "Default" password to match your own robot account
- sudo reboot
- Take a look at the default server https://www.vigibot.com
