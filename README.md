# Smart Drone

This is a personal challenge as a total newb, to create a drone that is as smart as I can possibly make it.

It will begin as a set of goals and milestones, I will be doing my best to document every nut and bolt. (Literally)


Initial equipmemt list will be as follows:

* Electrical Control Brain - Udoo x86 

* Artificial Intellgence Brain - Jetson TX2 Development kit

* Eyes (Color, Pattern) - Jetson TX2 Development kit CSI Camera

* Eyes (Depth, Distance) - Scanse Sweep

* Inner Ear? - Adafruit 10-Dof

* Test Body - S550 Kit

* Remote User Input - Turnigy X9 9-channel control (Heavily customized)

* Final Body - 8-rotor drone from scratch  (Final goal)

The project flow will be as follows:

- [x] Drone is assesmbled
- [x] Drone can be controlled directly with Turnigy X9
- [ ] Drone can self balance with Udoo / 10DoF
- [ ] Drone can communicate with laptop
- [ ] Drone can accept text commands
- [ ] Drone can accept / respond to text input (chatbot)
- [ ] Drone can see and hear (Image recognition, Audio recognition)
- [ ] Drone can learn from source material
- [ ] Drone can distinguish people
- [ ] Drone can distinguish voices



# Testing RX/TX

### RX Unit
https://user-images.githubusercontent.com/12061655/31365119-10c2d5de-ad1e-11e7-895c-21ea2a86d735.png

### Configuring the TX
I used the following motor mix for the hexacopter
![Rotor Setup](https://user-images.githubusercontent.com/12061655/31257565-19bca576-a9ee-11e7-8e64-d0e4d13e2339.png)
http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/frame-motor-mixing-table/

### Hooking it up to Arduino
Load https://github.com/csawtelle/ai-drone/blob/master/test_tx_read.ino to the Arduino
https://user-images.githubusercontent.com/12061655/31365089-ef51ac7c-ad1d-11e7-98b9-eaa387b6aef2.png

### Open Serial Monitor
You should see ~1300 when the throttle is down and ~2100 when the throttle is max 
