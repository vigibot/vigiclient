# Raspberry PI robot client for Vigibot.com

https://www.vigibot.com

### Installation on a clean Raspbian Stretch Lite

- Flash the last Raspbian Stretch Lite image: https://www.vigibot.com/raspbian

- Put your "wpa_supplicant.conf" and an empty "ssh" file inside the boot partition

- Connect to your Raspberry Pi via SSH

- sudo raspi-config

- Enable camera and I2C

- wget https://www.vigibot.com/vigiclient/install.sh

- sudo bash install.sh

- sudo nano /boot/robot.json

- Change "Demo" login and "Default" password to match your own robot account and reboot
