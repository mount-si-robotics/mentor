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

import org.firstinspires.ftc.mentor.common.ControllerMode;

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
@Disabled
public class MentorTeleOp extends LinearOpMode {

    /* Declare OpMode members. */

    private HardwareMentor robot = new HardwareMentor();   // Use mentor hardware definition

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double lastControllerSwitch = 0.0;

        boolean a_released = true;
        boolean b_released = true;
        boolean x_released = true;
        boolean y_released = true;
        boolean L3_released = true;
//        boolean R3_released = true;
//        boolean LeftBumper_released = true;
//        boolean RightBumper_released = true;
        boolean DpadLeft_released = true;
//        boolean DpadRight_released = true;
//        boolean DpadUp_released = true;
//        boolean DpadDown_released = true;
//        boolean Guide_released = true;
//        boolean Back_released = true;
//        boolean Start_released = true;

        // Initialize the robot hardware.
        robot.init(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
//        waitForStart();

        while (! isStarted()) {
            telemetry.addData(">>> ", "Press Start");
//            telemetry.addData("Alliance:")
        }

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

            if (robot.CONTROLLER_MODE == ControllerMode.TANK) {
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
            } else {
                left = -gamepad1.left_stick_y + -gamepad1.left_stick_x;
                right = -gamepad1.left_stick_y - -gamepad1.left_stick_x;
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if (robot.isSlowDrive()) {
                robot.drive(left / robot.getSlowDriveDivisor(), right / robot.getSlowDriveDivisor());
            } else {
                robot.drive(left, right);
            }

            ////////
            //
            // Debouncing buttons
            //
            // It is possible for buttons to be read multiple times through a loop, triggering
            // actions multiple times unintentionally.  Using a debounce routine fixes this issue
            // by ignoring further presses of a button until it has been released.
            //
            // Pattern:
            // Use a boolean variable to track whether a button has been released yet.
            // boolean x_released = true; // Button is not currently pressed
            // if (gamepad1.x) {  // X Button is pressed
            //     if (x_released) { if the x button is not currently pressed...
            //         x_released = false; // mark the button as not having been released yet
            //         // Do an action here
            //     }
            // }
            // else {
            //     x_released = true;

            if (gamepad1.a) {
                // A Button
                if (a_released) {
                    a_released = false;
                    robot.fireAndArmChooChooLauncher();
                }
            }
            else {
                a_released = true;
            }

            if (gamepad1.b) {
                // B Button
                // failsafe just in case
                if (b_released) {
                    b_released = false;
                    robot.stopChooChooMotor();
                }
            }
            else {
                b_released = true;
            }

            if (gamepad1.x) {
                // X Button

                if (x_released) {
                    x_released = false;
                    // Turn on the beater motor
                    if (!robot.isBeaterRunning) {
                        robot.startBeaterMotor();
                    } else {
                        robot.reverseBeaterMotor();
                    }
                }
            }
            else {
                x_released = true;
            }

            if (gamepad1.y) {
                if (y_released) {
                    y_released = false;
                    // Turn off the beater motor
                    robot.stopBeaterMotor();
                }
            }
            else {
                y_released = true;
            }

//            if (gamepad1.left_bumper) {
//                // TODO: left bumper
//                if (LeftBumper_released) {
//                }
//                else {
//                    LeftBumper_released = true;
//                }
//            }
//
//            if (gamepad1.right_bumper) {
//                // TODO: right bumper
//                if (RightBumper_released) {
//                }
//                else {
//                    RightBumper_released = true;
//                }
//            }

            if (gamepad1.left_stick_button) {
                // L3 Button

                if (L3_released) {
                    L3_released = false;
                    if (!robot.isSlowDrive()) {
                        robot.setSlowDrive(true);
                    } else {
                        robot.setSlowDrive(false);
                    }
                }
            }
            else {
                L3_released = true;
            }
//
//            if (gamepad1.right_stick_button) {
//                // R3 Button
//                if (R3_released) {
//                    R3_released = false;
//                }
//            }
//            else {
//                R3_released = true;
//            }

        if (gamepad1.dpad_left) {
            if (DpadLeft_released) {
                DpadLeft_released = false;
                // Wait 2 seconds before allowing another controller switch
                if ((robot.runtime.seconds() - lastControllerSwitch) > 2.0) {
                    if (robot.CONTROLLER_MODE == ControllerMode.TANK) {
                        robot.setControllerMode(ControllerMode.ARCADE);
                    } else {
                        robot.setControllerMode(ControllerMode.TANK);
                    }
                    lastControllerSwitch = robot.runtime.seconds();
                }
            }
        }
        else {
            DpadLeft_released = true;
        }

//        if (gamepad1.dpad_right) {
//            // DPad Right
//            if (DpadRight_released) {
//                DpadRight_released = false;
//                // Action
//
//            }
//        }
//        else {
//            DpadRight_released = true;
//        }

//        if (gamepad1.dpad_up) {
//            // DPad Up
//            if (DpadUp_released) {
//                DpadUp_released = false;
//                // Action
//
//            }
//        }
//        else {
//            DpadUp_released = true;
//        }

//            if (gamepad1.dpad_down) {
//                // DPad Down
//                if (DpadDown_released) {
//                    DpadDown_released = false;
//                    // Action
//
//                }
//            }
//            else {
//                DpadDown_released = true;
//            }

//            if (gamepad1.guide) {
//                // Guide Button
//                if (Guide_released) {
//                    Guide_released = false;
//                    // Action
//
//                }
//            }
//            else {
//                Guide_released = true;
//            }

//            if (gamepad1.back) {
//                // Back Button
//                if (Back_released) {
//                    Back_released = false;
//                    // Action
//
//                }
//            }
//            else {
//                Back_released = true;
//            }

//            if (gamepad1.start) {
//                // Start Button
//                if (Start_released) {
//                    Start_released = false;
//                    // Action
//
//                }
//            }
//            else {
//                Start_released = true;
//            }

//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
            telemetry.update();
        }
    }
}
