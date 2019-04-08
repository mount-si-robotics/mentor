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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.mentor.common.ControllerMode;

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

@TeleOp(name="MecanumTeleOp", group="Linear Opmode")
public class MecanumTeleOp extends LinearOpMode {

    /* Declare OpMode members. */

    private HardwareMecanum robot = new HardwareMecanum();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double rotation;
        double lastControllerSwitch = 0.0;

        // Initialize the robot hardware.
        robot.init(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (! isStarted()) {
            telemetry.addData(">>> ", "Press Start");
            telemetry.update();
        }

        //robot.runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());

            left = -gamepad1.left_stick_x;
            right = -gamepad1.left_stick_y;
            rotation = -gamepad1.right_stick_x;

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if (robot.isSlowDrive()) {
                robot.driveMecanumCartesian(left/robot.getSlowDriveDivisor(), right/robot.getSlowDriveDivisor(), rotation/robot.getSlowDriveDivisor());
            }
            else {
                robot.driveMecanumCartesian(left, right, rotation);
            }


            if (gamepad1.left_bumper) {
                if (! robot.isSlowDrive()) {
                    robot.setSlowDrive(true);
                }
                else {
                    robot.setSlowDrive(false);
                }
            }

            if (gamepad1.dpad_left) {

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

            telemetry.update();


        }
    }
}
