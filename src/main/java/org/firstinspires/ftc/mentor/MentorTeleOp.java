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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.max;
import static android.R.attr.right;
import static java.lang.Math.abs;

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

@TeleOp(name="MentorTeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class MentorTeleOp extends LinearOpMode {

    /* Declare OpMode members. */

    private HardwareMentor robot = new HardwareMentor();   // Use mentor hardware definition

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double lastControllerSwitch = 0.0;

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        robot.init(hardwareMap, this);

        gamepad1.setJoystickDeadzone(robot.DEADZONE);
        gamepad2.setJoystickDeadzone(robot.DEADZONE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.runtime.reset();

        // Zero out the gyro after starting the robot.
        // This corrects any gyro drift that may occur if the robot is idle for a long period of
        // time after you have pressed the Init button.  E.g. Announcer talks a long time,
        // FTA fixing a robot, Field maintenance, etc.
        robot.zeroGyro();

        /////
        //
        // Driver Control Mapping
        //
        // Controller 1 (Main)
        //
        // Left Stick = Left motor forward / back
        // Right Stick = Right motor forward / back
        //
        // Left Bumper = rotate left and follow line to beacon, push beacon if not alliance color
        // Right Bumper = rotate right and follow line to beacon, push beacon if not alliance color
        // Left + Right Bumper = Drive straight forward until line detected
        //
        // Left Pad = rotate 90 degrees to left
        // Right Pad = rotate 90 degrees to right
        // Up Pad = move forward one second
        // Down Pad = move forward one second
        //
        // A = Arm shooter
        // B = Fire shooter
        // X = Start ball gatherer
        // Y = Stop ball gatherer
        //
        // Stick buttons...
        // L3 =
        // R3 =

        // Controller 2

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());

            if (robot.CONTROLLER_MODE == HardwareMentor.ControllerMode.TANK) {
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
            }
            else {
                left = -gamepad1.left_stick_y + -gamepad1.left_stick_x;
                right = -gamepad1.left_stick_y - -gamepad1.left_stick_x;
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            robot.LBMotor.setPower(left);
            robot.RBMotor.setPower(right);

            if (gamepad1.a) {
                // TODO: A
                robot.ChooChooMotor.setPower(robot.CHOO_CHOO_MOTOR_SPEED);
            }

            if (gamepad1.b) {
                // TODO: B
                robot.ChooChooMotor.setPower(0.0);
            }

            if (gamepad1.x) {
                // Turn on the beater motor
                robot.BeaterMotor.setPower(robot.BEATER_MOTOR_SPEED);
            }

            if (gamepad1.y) {
                // Turn off the beater motor
                robot.BeaterMotor.setPower(0.0);
            }

//            if (gamepad1.left_bumper) {
//                // TODO: left bumper
//            }
//            if (gamepad1.right_bumper) {
//                // TODO: right bumper
//            }
//
//            if (gamepad1.left_stick_button) {
//                // TODO: L3
//            }
//
//            if (gamepad1.right_stick_button) {
//                // TODO: R3
//            }
//
            if (gamepad1.dpad_left) {

                // Wait 2 seconds before allowing another controller switch
                if ((robot.runtime.seconds() - lastControllerSwitch) > 2.0) {
                    if (robot.CONTROLLER_MODE == HardwareMentor.ControllerMode.TANK) {
                        robot.setControllerMode(HardwareMentor.ControllerMode.ARCADE);
                    } else {
                        robot.setControllerMode(HardwareMentor.ControllerMode.TANK);
                    }
                    lastControllerSwitch = robot.runtime.seconds();
                }
            }
//
//            if (gamepad1.dpad_right) {
//                // TODO: DR
//            }
//
//            if (gamepad1.dpad_up) {
//                // TODO: DU
//            }
//
//            if (gamepad1.dpad_down) {
//                // TODO: DD
//            }
//
//            if (gamepad1.guide) {
//                // TODO: back
//            }
//
//            if (gamepad1.back) {
//                // TODO: back
//            }
//
//            if (gamepad1.start) {
//                // TODO: start
//            }



//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
            telemetry.update();


        }
    }
}
