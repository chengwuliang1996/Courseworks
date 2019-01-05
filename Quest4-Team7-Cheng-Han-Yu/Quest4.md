#	Quest 4 Report

Authors: Wuliang Cheng, Qinglang Yu, Cong Han, 2018-11-30

##	Summary

In this quest we built up an autonomous car including two Lidar sensors, and a beacon that allows us to operate it to finish a round trip on a track autonomously. The car is able to receive the digital signal transmitting from the emiting station with the beacon. When the car receives 00, 01, 02, it will make a turn; when it receices 03, it will stop instantly. We also design a PID algorithm to allow the car to adjust its moving track when it is deviated from the path. 

Additionally, when we were doing the demo with Emily, we mentioned a particular issue with our Lidar. Since we are using two lidars, one at front for turing, and the other one is for PID self-adjusting, we found out that the side Lidar would mulfunction whenever it detects a "white wall". The testing track is made of a combination of white and black lego. We found out that the PID would not work whenever the side Lidar detects a white wall. However, it works fine with black wall. We reported this issue to Emily, and she witnessed our PID algorithm's problem with the "white wall". For quest 4, we demoed it to Emily, and she said that we met all the requirements on the rubics.
In the future quests, we will change the side lidar to IR and retesting the functionality. 

##	Evaluation Criteria

-We set up the lidars in our car, one on the front, and one on the side for measuring the distance from the wall.

-We set up IR TX/RX on the beacon to receive signals transmitting from the emitting station.

-We apply a PID algorithm to our autonomous car system so that the car can self-adjust the running track whenever it is deviated from the path.
	
	
##	Solution Design

Pins mapping:

- Vcc, GND, A0 for servo
- GPIO 26 for right wheel, GPIO 19 for left wheel PWM control
- GPIO 17 and 16 for front Lidar Uart connection
- GPIO 33 and 15 for side Lidar Uart connection
- GPIO 4 for IR beacon receiver
- Kp = 0.8, Ki = 0, Kd = 0.4, setpoint = 40(cm)



##	Modules, Tools, Source Used in Solution
<a href="./Codes/quest4.c">This is a link to the codes we used.</a>



##	Supporting Artifacts

[Here](https://drive.google.com/open?id=1kgTpnsMqmUOORpFJnNHpISBdKNc7PQDh) is a demo video of our quest. 