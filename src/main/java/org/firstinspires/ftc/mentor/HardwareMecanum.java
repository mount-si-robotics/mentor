package org.firstinspires.ftc.mentor;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.mentor.common.ControllerMode;
import org.firstinspires.ftc.mentor.common.LoggingMode;
import org.firstinspires.ftc.mentor.common.MotorMetadataMap;
import org.firstinspires.ftc.mentor.common.RobotConfigurationException;

import java.util.HashMap;
import java.util.Map;

import static java.lang.Math.abs;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  front drive motor:     "LFMotor"
 * Motor channel:  Right front drive motor:     "RFMotor"
 * Motor channel:  Left back drive motor:       "LBMotor"
 * Motor channel:  Right back drive motor:      "RBMotor"
 *
 *
 */

/*
 * TODO List
 *  Add Verbose logging mode to record function name, parameters to a log file. Add logging method.
 *  Different controller scaling for forward versus back.
 *  Implement RobotConfigurationExceptions during init
 *  Add support for a configuration file to allow dynamic changes to values instead of hardcoding
 *
 */

public class HardwareMecanum
{

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode LINEAR_OPMODE = null;

    // Maximum number of motors allowed by rules
    private static final int MAX_MOTORS_ALLOWED = 8;

    /* Public OpMode members. */
    private boolean DEBUG_MODE = false;  // Debugging is off by default

    private DcMotor  LFMotor   = null; // Left Front drive
    private DcMotor  RFMotor  = null;  // Right Front drive
    private DcMotor  LBMotor   = null; // Left Back drive
    private DcMotor  RBMotor  = null;  // Right Back drive

    // Maps to contain the motors allocated on this robot.
    // Add motors to the map(s) during init
    private Map<DcMotor, String> allMotorMap = new HashMap<>();
    private Map<DcMotor, String> driveMotorMap = new HashMap<>();

    private MotorMetadataMap motorMetadataMap = new MotorMetadataMap();

    private float DEFAULT_DEADZONE = 0.05f;
    private boolean isSlowDrive = false;
    private double slowDriveDivisor = 2; // Divide speed by this number if in slow speed mode.
                                         // Dividing by 2 means speed is half the max speed

    // Motor Constants
    private static final double MAX_MOTOR_SPEED = 1.0;


    // Sane defaults
    public ControllerMode CONTROLLER_MODE = ControllerMode.TANK;

    ///////////////////////////////////////////////////
    ///////////////////////////////////////////////////

    /* Constructor */
    HardwareMecanum(){
    }

    // Get a handle to the OpMode that is using this class
    private void setLinearOpMode(LinearOpMode opMode) {
        String functionName = "setLinearOpMode";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        LINEAR_OPMODE = opMode;
    }

    void setLoggingMode(LoggingMode loggingMode) {
        String functionName = "setLoggingMode";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }
    }

    // Set the game pad dead zone
    void setGamepadDeadzone(float deadzone) {
        String functionName = "setGamepadDeadzone";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        if (LINEAR_OPMODE != null) {
            LINEAR_OPMODE.gamepad1.setJoystickDeadzone(deadzone);
        }

        try {
            throw(new RobotConfigurationException(functionName + ": Must be LinearOpMode"));
        } catch (RobotConfigurationException e) {
            e.printStackTrace();
        }
    }

    // Set the game pad dead zone to the default value
    public void setGamepadDeadzone() {
        String functionName = "setGamepadDeadzone";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        setGamepadDeadzone(DEFAULT_DEADZONE);
    }

    void setControllerMode(ControllerMode controllerMode) {
        String functionName = "setControllerMode";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        CONTROLLER_MODE = controllerMode;
    }


    public boolean isSlowDrive() {
        String functionName = "isSlowDrive";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        return isSlowDrive;
    }

    public void setSlowDrive(boolean slow) {
        String functionName = "setSlowDrive";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        isSlowDrive = slow;
    }

    public double getSlowDriveDivisor() {
        String functionName = "getSlowDriveDivisor";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        return slowDriveDivisor;
    }

    ////////
    //
    // Drive methods
    //
    ////////

    // Drive routine that supports Mecanum wheels.  Will maintain a gyro heading.
    public void driveMecanumCartesian (double x, double y, double rotation, double gyroHeading, boolean inverted) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

//        // Make sure x and y are in range of -1.0 to 1.0
//        clip(x, -1.0f, 1.0f);
//        clip(y, -1.0f, 1.0f);
//        clip(rotation, -1.0f, 1.0f);

        if (inverted) {
            x = -x;
            y = -y;
        }

        double cosA = Math.cos(Math.toRadians(gyroHeading));
        double sinA = Math.sin(Math.toRadians(gyroHeading));
        x = x * cosA - y * sinA;
        y = x * sinA + y * cosA;

        LFPower = x + y + rotation;
        RFPower = -x + y - rotation;
        LBPower = -x + y + rotation;
        RBPower = x + y - rotation;

        // Normalize the power of each motor so that the maximum motor power is 1.0
        double maxPower = Math.max(abs(LFPower), Math.max(abs(RFPower), Math.max(abs(LBPower), abs(RBPower))));
        if (maxPower > MAX_MOTOR_SPEED) {
            LFPower /= maxPower;
            RFPower /= maxPower;
            LBPower /= maxPower;
            RBPower /= maxPower;
        }

        if (LFMotor != null) {
            LFMotor.setPower(LFPower);
        }
        if (RFMotor != null) {
            RFMotor.setPower(RFPower);
        }
        if (LBMotor != null) {
            LBMotor.setPower(LBPower);
        }
        if (RBMotor != null) {
            RBMotor.setPower(RBPower);
        }

    }

    // Mecanum drive routing that uses a 0 degree gyro setting
    public void driveMecanumCartesian (double x, double y, double rotation, boolean inverted) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        driveMecanumCartesian (x, y, rotation, 0.0, inverted);
    }

    // Mecanum drive routine that supports rotation, 0 degree gyro setting, no inversion
    public void driveMecanumCartesian (double x, double y, double rotation) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        driveMecanumCartesian (x, y, rotation, 0.0, false);
    }

    // Mecanum drive that support only x,y, no rotation, gyro heading or inversion
    public void driveMecanumCartesian (double x, double y) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        driveMecanumCartesian (x, y, 0.0, 0.0, false);
    }

    // Stop drive motors
    void stopDriveMotors() {
        String functionName = "stopDriveMotors";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setPower(0.0);
        }
    }

    // Set the mode for the drive motors
    void setDriveMotorMode(DcMotor.RunMode runMode) {
        String functionName = "setDriveMotorMode";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setMode(runMode);
        }
    }

    // Set the zero power behavior for drive motors
    private void setDriveMotorZeroPowerBehavior() {
        String functionName = "setDriveMotorZeroPowerBehavior";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    ////////
    //
    // Initialization
    //
    ////////

    // Initialize motors
    private void initializeMotors() throws RobotConfigurationException {
        String functionName = "initializeMotors";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        // Initialize the motor metadata map
        motorMetadataMap.initialize();

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }

        // Define Motors
        LFMotor   = hwMap.dcMotor.get("front_left");
        RFMotor  = hwMap.dcMotor.get("front_right");
        LBMotor   = hwMap.dcMotor.get("back_left");
        RBMotor  = hwMap.dcMotor.get("back_right");


        // All Motor Map
        allMotorMap.put(LFMotor, "front_left");
        allMotorMap.put(RFMotor, "front_right");
        allMotorMap.put(LBMotor, "back_left");
        allMotorMap.put(RBMotor, "back_right");


        // Drive motor map
        driveMotorMap.put(LFMotor, "front_left");
        driveMotorMap.put(RFMotor, "front_right");
        driveMotorMap.put(LBMotor, "back_left");
        driveMotorMap.put(RBMotor, "back_right");


        // Set starting motor directions
        LFMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RFMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LBMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RBMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set braking mode for drive motor
        // SDK default for year 2 is to put the motor in BRAKE mode.  Year 1 was FLOAT.
        setDriveMotorZeroPowerBehavior();

        // Set drive motors to zero power
        stopDriveMotors();

        // Validate the number of motors is less than or equal to the maximum number allowed
        if (allMotorMap.size() > MAX_MOTORS_ALLOWED) {
            throw new RobotConfigurationException("Too many motors configured!");
        }
    }

    // Initialize Hardware interfaces
    // Use this version with non-LinearOpModes
    public void init(HardwareMap ahwMap) {
        String functionName = "init";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        setLoggingMode(LoggingMode.NONE);

        // Save reference to Hardware map
        hwMap = ahwMap;

        setGamepadDeadzone();

        try {
            initializeMotors();
        }
        catch (RobotConfigurationException rce) {
        }
    }

    // Initialize Hardware interfaces
    public void init(HardwareMap ahwMap, LinearOpMode om) {
        String functionName = "init";

        if (DEBUG_MODE) {
            RobotLog.d("%s", functionName);
        }

        setLinearOpMode(om);
        init(ahwMap);

    }
}

