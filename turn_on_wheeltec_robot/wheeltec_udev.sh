echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0666", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar_laser"' >/etc/udev/rules.d/rplidar_laser.rules

service udev reload
sleep 2
service udev restart


