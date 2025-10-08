#install.sh
Password="503814"

#change mode
chmod 777 /home/zhuo-skadi/Documents/berxel-sdk-master/berxel-usb.rules

#move rules to /etc/udev/rules.d
echo $Password | sudo -S cp /home/zhuo-skadi/Documents/berxel-sdk-master/berxel-usb.rules /etc/udev/rules.d
echo $Password | sudo -S udevadm control --reload-rules
