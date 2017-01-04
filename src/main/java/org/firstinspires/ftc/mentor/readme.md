## Configuring a robot

All robot hardware definitions and routines are placed into a Hardware[RobotName] Java class.

**Step 1:** Attach the robot controller phone to the robot and create a new hardware configuration xml
file. Make sure you use all lower case letters for the name of the configuration.

**Step 2:** Copy the robot configuration xml file from the phone and place it into the res/xml directory
in this project.  This will install the configuration file every time your app is installed,
ensuring that the correct configuration file is installed on the robot controller phone.

**Step 3:** Confirm which motors are being used to drive your robot and enable them in the init
routine. Make sure to set the appropriate motor type for each motor (e.g. Tetrix / AndyMark / etc.).
Here is the order of motors (LF = Left Front, LM = Left Middle, RB = Right Back, etc.):

  _Front of robot_

LFMotor -- RFMotor

LMMotor -- RMMotor

LBMotor -- RBMotor

  _Back of robot_

**Step 4:** Confirm which sensors your robot is going to use.

**Step 5:** Confirm which servos your robot is going to use and add any initialization routines needed.

**Step 6:** Check all of the various constants to ensure they are appropriate for your robot.

**Step 7:** Test!