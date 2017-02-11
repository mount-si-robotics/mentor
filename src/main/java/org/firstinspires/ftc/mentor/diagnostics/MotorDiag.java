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
package org.firstinspires.ftc.mentor.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="MotorDiagnostic", group="Diagnostics")  // @Autonomous(...) is the other common choice
//@Disabled
public class MotorDiag extends LinearOpMode {

    private static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    private static final int    CYCLE_MS    =   50;     // period of each cycle
    private static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    private static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    private static final double ENC_TEST_POWER = 0.30;
    private static final int ENC_COUNT_UP_TARGET = 3000;
    private static final int ENC_COUNT_DOWN_TARGET = -3000;
    private static final double ENC_TEST_TIMEOUT_MS = 10.0 * 1000;

    // Define class members
    private DcMotor motor;
    private double  power   = 0;
    boolean rampUp  = true;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testMotor = null;


    private void displayMotorTelemetry(String testName) {
        // Display the current value
        telemetry.addData("Testing", testName);
        telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Motor Power", "%5.2f", power);
        telemetry.addData("Motor encoder value", "%.2f", motor.getCurrentPosition());
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();
    }

    private void rampUp() {
        String functionName = "rampUp";

        while (opModeIsActive() && (power < MAX_FWD)) {
            power += INCREMENT;
            if (power >= MAX_FWD) {
                power = MAX_FWD;
            }

            displayMotorTelemetry(functionName);

            // Set the motor to the new power and pause;
            motor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        motor.setPower(0);
    }

    private void rampDown() {
        String functionName = "rampDown";

        while (opModeIsActive() && (power > MAX_REV)) {
            power -= INCREMENT;
            if (power <= MAX_REV) {
                power = MAX_REV;
            }

            displayMotorTelemetry(functionName);

            // Set the motor to the new power and pause;
            motor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        motor.setPower(0);
    }

    private void runToPositionUp() {
        String functionName = "runToPositionUp";

        double startTime = System.currentTimeMillis();
        double currTime = startTime;
        DcMotor.RunMode mode;

        // Save the current motor mode
        mode = motor.getMode();

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ENC_COUNT_UP_TARGET);
        motor.setPower(ENC_TEST_POWER);

        while (opModeIsActive() && ((currTime - startTime) < ENC_TEST_TIMEOUT_MS)) {
            displayMotorTelemetry(functionName);
            currTime = System.currentTimeMillis();
        }

        // Stop motor just in case
        motor.setPower(0);

        // Restore mode
        motor.setMode(mode);
    }

    private void runToPositionDown() {
        String functionName = "runToPositionDown";

        double startTime = System.currentTimeMillis();
        double currTime = startTime;
        DcMotor.RunMode mode;

        // Save the current motor mode
        mode = motor.getMode();

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ENC_COUNT_DOWN_TARGET);
        motor.setPower(ENC_TEST_POWER);

        while (opModeIsActive() && ((currTime - startTime) < ENC_TEST_TIMEOUT_MS)) {
            displayMotorTelemetry(functionName);
            currTime = System.currentTimeMillis();
        }

        // Stop motor just in case
        motor.setPower(0);

        // Restore mode
        motor.setMode(mode);
    }

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("test_motor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
        while (! isStarted()) {
            // wait for start
            telemetry.addData(">>>", "Press start");
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            rampUp();

            rampDown();

            runToPositionUp();

            runToPositionDown();
        }
    }


}
