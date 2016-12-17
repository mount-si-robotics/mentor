/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.mentor;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MentorHardwareTestOpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class MentorHardwareTestOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMentor robot = new HardwareMentor();  // Use mentor hardware definition

    private ElapsedTime runtime = new ElapsedTime();

    MentorHardwareTestConfiguration mentorHardwareTestConfiguration = new MentorHardwareTestConfiguration();
    String menuConfigurationFile = "HardwareTest.json";

    public void writeMenuConfigurationFile() {

    }

    public void readMenuConfigurationFile() {

    }

    public void testListHeadings() {

    }

    public void testGetLoggingMode() {
        HardwareMentor.LoggingMode loggingMode = robot.getLoggingMode();
        telemetry.addData("testGetLoggingMode: LoggingMode = ", loggingMode.toString());
        telemetry.update();
    }

    public void testSetLoggingMode() {
        String functionName = "testSetLoggingMode";
        robot.setLoggingMode(HardwareMentor.LoggingMode.VERBOSE);
        testGetLoggingMode();
    }

    public void testGetControllerMode() {
        String functionName = "testGetControllerMode";

        HardwareMentor.ControllerMode controllerMode = robot.getControllerMode();
        telemetry.addData(functionName + ": Controller Mode = ", controllerMode.toString());
        telemetry.update();
    }

    public void testSetControllerMode() {
        String functionName = "testSetControllerMode";

        robot.setControllerMode(HardwareMentor.ControllerMode.ARCADE);
        testGetControllerMode();
    }

    public void testGetScaleMode() {
        String functionName = "testGetScaleMode";

        HardwareMentor.ScaleMode scaleMode = robot.getScaleMode();
        telemetry.addData(functionName + ": scale mode = ", scaleMode);
        telemetry.update();
    }

    public void testSetScaleMode() {
        String functionName = "testSetScaleMode";

        robot.setScaleMode(HardwareMentor.ScaleMode.CUBE);
        testGetScaleMode();
    }

    public void testScaleValue() {
        String functionName = "testScaleValue";

        telemetry.addData(functionName + ": linear 0.5", robot.scaleValue(0.5, HardwareMentor.ScaleMode.LINEAR));
        telemetry.addData(functionName + ": square 0.5", robot.scaleValue(0.5, HardwareMentor.ScaleMode.SQUARE));
        telemetry.addData(functionName + ": cube 0.5", robot.scaleValue(0.5, HardwareMentor.ScaleMode.CUBE));
        telemetry.update();
    }

    public void testGetDriveTrain() {
        String functionName = "testGetDriveTrain";

        HardwareMentor.DriveTrain driveTrain = robot.getDriveTrain();
        telemetry.addData(functionName + ": drive train = ", driveTrain);
        telemetry.update();
    }

    public void testSetDriveTrain() {
        String functionName = "testSetDriveTrain";

        robot.setDriveTrain(HardwareMentor.DriveTrain.TWO_WHEEL_CENTER);
        testGetDriveTrain();
    }

    public void testGetDebugMode() {
        String functionName = "testGetDebugMode";

        telemetry.addData(functionName + ": debug mode = ", robot.getDebugMode());
        telemetry.update();
    }

    public void testSetDebugMode() {
        String functionName = "testSetDebugMode";

        robot.setDebugMode(true);
        testGetDebugMode();

        robot.setDebugMode(false);
    }




    @Override
    public void runOpMode() {

        // Initialize robot hardware
        //robot.init(hardwareMap);

        // Set the default gamepad deadzone
        gamepad1.setJoystickDeadzone(robot.DEADZONE);
        gamepad2.setJoystickDeadzone(robot.DEADZONE);

        // TODO: Read menu configuration file
        //mentorHardwareTestConfiguration = new MentorHardwareTestConfiguration().deserialize("");

        //mentorHardwareTestConfiguration.functionTests.add("getLoggingMode");
        //mentorHardwareTestConfiguration.functionTests.add("setLoggingMode");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Zero out the gyro after starting the robot.
        // This corrects any gyro drift that may occur if the robot is idle for a long period of
        // time after you have pressed the Init button.  E.g. Announcer talks a long time,
        // FTA fixing a robot, Field maintenance, etc.
        robot.zeroGyro();

        ////////
        //
        // Function Tests
        //
        ////////

//        testGetLoggingMode();
//        testSetLoggingMode();
//        testGetControllerMode();
//        testSetControllerMode();
//        testGetScaleMode();
//        testSetScaleMode();
//        testScaleValue();
//        testGetDriveTrain();
//        testSetDriveTrain();
//        testGetDebugMode();
//        testSetDebugMode();

        ////////
        //
        // Motor Tests
        //
        ////////

        ////////
        //
        // Servo Tests
        //
        ////////

        ////////
        //
        // Sensor Tests
        //
        ////////



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                testSetDebugMode();
            }

//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
