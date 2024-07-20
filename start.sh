sudo uhubctl -l 2 -a cycle -p 1-4
sleep 5
sudo chmod 777 /dev/ttyUSB0
sleep 1
gnome-terminal -x /home/stardust/custom_controller/udp2uart.out
sudo python3 /home/stardust/custom_controller/main.py
