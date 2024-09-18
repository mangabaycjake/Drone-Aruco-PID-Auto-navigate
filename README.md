# Drone-Aruco-PID-Autonavigate
*My undergraduate thesis* - A raspberry pi program that autonomously navigates a drone horizontally.

<div style="display: flex;">
  <img src="Documentations/SetupDrone1.jpg" alt="Att Img 1" style="width: 45%; margin-right: 5%;"/>
  <img src="Documentations/SetupDrone2.jpg" alt="Att Img 2" style="width: 45%;"/>
</div>

##  
A Raspberry Pi Zero 2W is attached to view the path and send commands to a Ryze Tello drone. The drone follows the placement of the ArUco markers while PID smoothens the movement. The placing of rpi and batter was inspired from [erviveksoni's post](https://github.com/erviveksoni/raspberrypi-controlled-tello).

## Flight Samples
Two samples of actual flight as seen from the pi cam can be accessed [here](https://github.com/mangabaycjake/Drone-Aruco-PID-Autonavigate/tree/main/Documentations).

<div style="display: flex;">
  <img src="Documentations/output_video_19.725_0.0832.gif" alt="Fly Vid 1" style="width: 100%; margin-right: 10%;"/>
  <img src="Documentations/output_video_20.882_0.0832.gif" alt="Fly Vid 2" style="width: 100%;"/>
</div>

## Hardware Setup
- Pi Camera Module V2 installed on the RPi captures images
- Separate [battery](https://www.lazada.com.ph/products/i2667786599-s12702717118.html) is used
- 5V [power module](https://www.lazada.com.ph/products/i1309980104-s4775296337.html)
 is used for charging and stepping up voltage

> ![Hardware diagram](https://github.com/user-attachments/assets/ec4cab3a-9a4a-4136-bec7-e196cbd837f9)

## Bluetooth Terminal
Bluetooth terminal is used with a phone to initiate the start of the program and the abortion to ensure drone safety.

> <img src="Documentations/Bluetooth.jpg" width=200></img>

## Output CSV
An output CSV [(sample)](Documentations/pos_20.882_0.0832.csv) is generated for analytics.

## Notes
- Use virtual environment when implementing in raspberry pi due to library coflicts
- For other less supported libraries (esp. opencv), try earlier versions.

## Installed Libraries
> | Package                | Version   |
> |------------------------|-----------|
> | av                     | 8.0.0     |
> | djitellopy             | 2.5.0     |
> | numpy                  | 1.26.4    |
> | opencv-contrib-python  | 4.5.3.56  |
> | opencv-python-headless | 4.5.3.56  |
> | pillow                 | 10.3.0    |
> | pip                    | 24.0      |
> | pkg_resources          | 0.0.0     |
> | PyBluez                | 0.23      |
> | setuptools             | 44.1.1    |
> | simple-pid             | 2.0.0     |
> | wheel                  | 0.43.0    |


