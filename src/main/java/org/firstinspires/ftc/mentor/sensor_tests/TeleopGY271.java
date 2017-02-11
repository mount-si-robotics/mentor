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
package org.firstinspires.ftc.mentor.sensor_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.mentor.common.DataLogger;
import org.firstinspires.ftc.mentor.sensors.HMC5883LGY271;

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

@TeleOp(name="GY271", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleopGY271 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private I2cDeviceSynch gy271Device;
    private HMC5883LGY271 gy271;

    private DataLogger dataLogger;

    private byte IDA;
    private byte IDB;
    private byte IDC;

    private int testNum = 0;
    boolean done = false;

    private void logBool(String title, boolean data) {
        telemetry.addData(title, "%s\n", data);
        dataLogger.logData("%s: %s\n", title, data);
    }

    private void logByte(String title, byte data) {
        telemetry.addData(title, "0x0%2x\n", data);
        dataLogger.logData("%s: 0x0%2x\n", title, data);

    }

    private void logChar(String title, char data) {
        telemetry.addData(title, "%c\n", data);
        dataLogger.logData("%s: %c\n", title, data);
    }

    private void logInt(String title, int data) {
        telemetry.addData(title, "%d\n", data);
        dataLogger.logData("%s: %d\n", title, data);
    }

    private void logString(String title, String data) {
        telemetry.addData(title, "%s\n", data);
        dataLogger.logData("%s: %s\n", title, data);

    }

    private void test_getID_ABC() {
        IDA = gy271.getIDA();
        IDB = gy271.getIDB();
        IDC = gy271.getIDC();

        logChar("IDA", (char)IDA);
        logChar("IDB", (char)IDB);
        logChar("IDC", (char)IDC);
    }

    private void test_deviceInfo() {
        String deviceName = gy271.getDeviceName();
        HardwareDevice.Manufacturer manufacturer = gy271.getManufacturer();

        logString("Device Name", deviceName);
        logString("Manufacturer", manufacturer.toString());
    }

    private void test_getSampleAveraging() {
        byte sampleAveraging = gy271.getSampleAveraging();

        logByte("Sample Averaging", sampleAveraging);

    }

    private void test_getDataRate() {
        byte dataRate = gy271.getDataRate();

        logByte("DataRate", dataRate);
    }

    private void test_getMeasurementBias() {
        byte mBias = gy271.getMeasurementBias();

        logByte("MeasurementBias", mBias);
    }

    private void test_getGain() {
        byte gain = gy271.getGain();

        logByte("Gain", gain);
    }

    private void test_getMode() {
        byte mode = gy271.getMode();

        logByte("Mode", mode);
    }

    private void test_getHeading() {
        // TODO: implement
        logString("Heading", "unimplemented!");
    }

    private void test_getHeadingX() {
        short heading = gy271.getHeadingX();

        logInt("HeadingX", heading);
    }

    private void test_getHeadingY() {
        short heading = gy271.getHeadingY();

        logInt("HeadingY", heading);
    }

    private void test_getHeadingZ() {
        short heading = gy271.getHeadingZ();

        logInt("HeadingZ", heading);
    }

    private void test_getLockStatus() {
        boolean status = gy271.getLockStatus();

        logBool("LockStatus", status);
    }

    private void test_getReadyStatus() {
        boolean status = gy271.getReadyStatus();

        logBool("ReadyStatus", status);
    }




    @Override
    public void runOpMode() {
        dataLogger = DataLogger.getInstance();
        dataLogger.initialize(this);

        gy271Device = hardwareMap.i2cDeviceSynch.get("gy");
        gy271 = new HMC5883LGY271(gy271Device, false);
        gy271.doInitialize();

        gy271.testConnection();

        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
        while (! isStarted() ) {
            // Wait for start
            telemetry.addData(">>>", "Press START");
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Direction:", gy271.getDirection());
            telemetry.addData("X:", gy271.getHeadingX());
            telemetry.addData("Y:", gy271.getHeadingY());
            telemetry.addData("Z:", gy271.getHeadingZ());

            switch (testNum) {
                case 1:
                    test_getID_ABC();
                    break;
                case 2:
                    test_deviceInfo();
                    break;
                case 3:
                    test_getSampleAveraging();
                    break;
                case 4:
                    test_getDataRate();
                    break;
                case 5:
                    test_getMeasurementBias();
                    break;
                case 6:
                    test_getGain();
                    break;
                case 7:
                    test_getMode();
                    break;
                case 8:
                    test_getHeading();
                    break;
                case 9:
                    test_getHeadingX();
                    break;
                case 10:
                    test_getHeadingY();
                    break;
                case 11:
                    test_getHeadingZ();
                    break;
                case 12:
                    test_getLockStatus();
                    break;
                case 13:
                    test_getReadyStatus();
                    break;
                default:
                    done = true;
                    break;
            }

            testNum++;
            telemetry.update();
        }
        gy271.close();
        dataLogger.close();
    }
}
