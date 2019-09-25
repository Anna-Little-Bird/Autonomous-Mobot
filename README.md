# Autonomous Mobot

![mobot](https://github.com/Anna-Little-Bird/Autonomous-Mobot/blob/master/Photos/1.JPG)

The aim of this project was to design the Autonomous Robot (or the Mobot) based on Arduino Uno board. The Mobot has to perform a set of tasks:
- move around the specified area avoiding hitting the obstacles;
- detect the blue cube within the working area;
- grab the blue cube and move it outside the area.

For detecting the colour of the object and the border of the area two I2C sensors were used: TCS34725 and APDS-9960 respectively. The border control was implemented by built-in IR part of APDS-9960 sensor using interrupts.

For obstacle avoidance an ultrasonic sensor HCSR04 was used. In order to control this sensor "New Ping" library was installed. The ulrasonic sensor was mounted on the SG-90 servo motor to perform scanning.

The moving part consisted of motor shield L298N and 4 wheels chassis.

To implement the above requirements the following state diagram was designed:
![state diagram](https://github.com/Anna-Little-Bird/Autonomous-Mobot/blob/master/Photos/State%20Diagram.jpg)

At the start of the program the Mobot goes into SCAN state. In this state it scans the area in front of it within 120° by rotating ultrasonic sensor to 5 angles: 30°, 60°, 90°, 120° and 150°. Due to the errors from ultrasonic sensor the measurements are performed 2 times. If the measured distance at any angle is less than 35cm the Mobot goes into POSITION state. Otherwise it will continue scanning the surface.

The aim of POSITION state is to align the Mobot to the centre of the object so the further colour detection will be more accurate. In this state the Mobot performs left and right turns with different delays depending on the angle from SCAN state. The aligning process continues until the angle equals to 90°. After that the Mobot goes into COLOUR state. If while aligning the Mobot loses the object from its view, i.e. the distance towards the object becomes more than 35cm or the object goes into blind zone, the Mobot then moves backwards and goes back into SCAN state to ensure to find the object again.

In COLOUR state the Mobot approaches the object to detect its colour. Due to very short colour detection distance (less than 1cm) the value of the delay depends on the distance towards the object to try to avoid hitting the object. If the colour of the object is not blue AVOID state executes. In this state the obstacle avoiding part of the programme is performed. After avoiding the obstacle the Mobot goes back into SCAN state.

If the colour of the object is blue then the robot goes into GRAB state where it grabs the object and goes into SCAN state. The program sets the variable *object* to true. If during these conditions (SCAN state and *object*=true) the Mobot detects something this will be considered as the obstacle. Thus the robot will go into AVOID state immediately and after avoiding will come back to SCAN state. It will stay in SCAN state until it meets the border.

Every state can be interrupted by interrupt service routine *border_control* which can be initiated by IR part of APDS-9960 sensor. When interrupt occurs the program checks the state of *object* variable. If it is false then the Mobot moves backwards and then right to avoid the border and stay inside the working area. If this variable is true Mobot moves forward for 0.5 sec and then goes into STOP state. In this state Mobot completely stops, gripper’s arms are open and the program finishes.
