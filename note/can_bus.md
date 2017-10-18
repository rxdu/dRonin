## CAN Bus Commands

```
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyACM0
sudo ip link set up slcan0
```

-s6 means CAN bitrate 500kbit/s, -s8 means 1Mbit/s. -S parameter is used to setup serial speed to the USB-to-serial controller in the USB-to-CAN cable. The full CAN bitrate table for slcan can be found here.

Reference:

* https://stackoverflow.com/questions/41137320/how-to-communicate-with-can-with-slcan