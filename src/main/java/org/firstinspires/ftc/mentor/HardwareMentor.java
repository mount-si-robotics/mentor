package org.firstinspires.ftc.mentor;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.nextAfter;
import static java.lang.Thread.sleep;
import com.google.gson.Gson;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

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
 * Motor channel:  Choo Choo Launcher motor:    "ChooChooMotor"
 * Motor channel:  Beater Bar motor:            "BeaterMotor"
 *
 * Servo channel:
 *
 * Sensor: I2C: Gyroscope:            "gyro"
 * Sensor: I2C: IMU:                  "imu"
 * Sensor: I2C: Modern Robotics Color:"color"
 * Sensor: I2C: Adafruit Color:       "afcolor"
 * Sensor: Analog 0: Modern Robotics Optical Distance Sensor: "ods"
 * Sensor: Digital channel 0:         "alliance"
 * Sensor: Digital channel 1:         "startposition"
 * Sensor: Digital channel 2:         "strategy"
 * Sensor: Digital channel 3:         "irBeamBreak"
 * Sensor: Digital channel 5:         (Adafruit Color Sensor LED pin)
 *
 */

/*
 * TODO List
 *  Add Verbose logging mode to record function name, parameters to a log file. Add logging method.
 *  Different controller scaling for forward versus back.
 *  Implement RobotConfigurationExceptions during init
 *  Implement a generic RobotException for other types of unexpected errors.
 *  Add support for a configuration file to allow dynamic changes to values instead of hardcoding
 *
 */

public class HardwareMentor
{
    // Class name for logging purposes
    private String className = "HardwareMentor";

    private OpMode OPMODE = null;

    /* Public OpMode members. */
    public boolean DEBUG_MODE = false;  // Debugging is off by default

    public DcMotor  LFMotor   = null;
    public DcMotor  RFMotor  = null;
    public DcMotor  LBMotor   = null;
    public DcMotor  RBMotor  = null;
    public DcMotor  ChooChooMotor = null;
    public DcMotor  BeaterMotor = null;

    MotorMetadataMap motorMetadataMap = new MotorMetadataMap();

    // Mapping occurs in initializeMotors()
    Map<DcMotor, MotorMetadata> motorData = new HashMap<>();

    public float DEADZONE = 0.05f;
    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    double APPROACH_SPEED = 0.5; // drive speed

    // Motor Constants
    static final double MIN_MOTOR_SPEED = -1.0;
    static final double MAX_MOTOR_SPEED = 1.0;

    // Rotation speed and direction for the choo choo launcher motor
    double CHOO_CHOO_MOTOR_SPEED = 0.5;

    // Rotation speed and direction for the beater (ball collector) motor
    double BEATER_MOTOR_SPEED = 0.5;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    double ENCODER_DEGREE_PER_COUNT = 360.0 / COUNTS_PER_MOTOR_REV;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public DeviceInterfaceModule cdim = null;
    public GyroSensor gyro = null;  // Modern Robotics Gyroscope
    public ColorSensor color = null; // Modern Robotics Color
    public ColorSensor afcolor = null; // Adafruit Color Sensor

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int ADAFRUIT_COLOR_LED_CHANNEL = 5;

    public OpticalDistanceSensor ods = null;  // Modern Robotics ODS
    public BNO055IMU imu;  // Adafruit IMU

    // Hardware switches for selecting runtime parameters
    public DigitalChannel allianceChannel = null;
    public DigitalChannel startPositionChannel = null;
    public DigitalChannel strategyChannel = null;

    // IR Beam Break Digital Sensor
    public DigitalChannel irBeamBreak = null;

    // Potentiometer for providing analog input
    public AnalogInput configurationPot = null;

    // Number of ranges provided by potentiometer.  Must be a power of 2.
    // Note: Choice of potentiometer type should be linear, not log.
    // 16 = 4 on/off values (4 bits)
    // 8 = 3 on/off values (3 bits)
    // 4 = 2 on/off values (2 bits)
    // 2 = on/off (1 bit) -- just use a switch...
    // Other uses:
    // Division value. E.g. If pot is in division 7, then pause 7 seconds.
    // Raw value.  Returns the raw value of the pot; doesn't convert to a division.
    private int POT_DIVISIONS = 16;
    private boolean[] potBitValues = new boolean[(POT_DIVISIONS >> 1)];
    private int potDivision = 0;
    private double potRawValue = 0.0;

    // Enums
    public enum Alliance {
        RED,
        BLUE
    }

    public enum Strategy {
        STRATEGY1,
        STRATEGY2
    }

    public enum StartPosition {
        POSITION1,
        POSITION2
    }

    public enum LoggingMode {
        VERBOSE,
        NORMAL,
        NONE
    }

    // TODO: Implement generic drive routine that calls the appropriate drive function
    public enum DriveTrain {
        TWO_WHEEL_REAR,
        TWO_WHEEL_CENTER,
        TWO_WHEEL_FRONT,
        FOUR_WHEEL,
        MECANUM_CARTESIAN,
        MECANUM_POLAR,
        NONE,
        NULL // Will simulate motor movement for debugging  TODO: Implement NULL mode for motors
    }

    enum ScaleMode {
        LINEAR,
        ONE_PT_FIVE,
        ONE_PT_SEVEN,
        SQUARE,
        CUBE
    }

    enum ControllerMode {
        TANK,
        ARCADE
    }



    // Sane defaults
    // TODO: Throw exception somewhere if DriveTrain.NONE is configured
    public DriveTrain DRIVE_TRAIN = DriveTrain.NONE;
    public ScaleMode SCALE_MODE = ScaleMode.LINEAR;
    public ControllerMode CONTROLLER_MODE = ControllerMode.TANK;
    private LoggingMode LOGGING_MODE = LoggingMode.NONE;

    // TODO: better name for log file
    private String LOGGING_FILE = "logfile";

    static final int    BLUE_LED    = 0;     // Blue LED channel on DIM
    static final int    RED_LED     = 1;     // Red LED Channel on DIM

    public Alliance alliance = null;
    public Strategy strategy = null;
    public StartPosition startPosition = null;

    // State used for updating telemetry
//    Orientation angles;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Configuration file for overriding default settings
    public MentorHardwareRobotConfiguration robotConfigurationData = null;
    public String robotConfigurationDataFile = null;



    ///////////////////////////////////////////////////
    ///////////////////////////////////////////////////

    /* Constructor */
    public HardwareMentor(){
        String functionName = "HardwareMentor";
        // Empty constructor
        // Nothing to see here...
    }

    // Get a handle to the OpMode that is using this class
    public void setOpMode(OpMode opMode) {
        OPMODE = opMode;
    }

    // Get the currently set LoggingMode
    public LoggingMode getLoggingMode () {
        String functionName = "getLoggingMode";

        return LOGGING_MODE;
    }

    public void setLoggingMode (LoggingMode loggingMode) {
        // TODO: NOT TESTED YET
        String functionName = "setLoggingMode";

        if (loggingMode != LOGGING_MODE) {
            // TODO: Handle state change
            if (loggingMode == LoggingMode.NONE) {
                // TODO: stop logging
            }
            else {
                // TODO: open log file and start logging
            }

            LOGGING_MODE = loggingMode;
        }
    }

    public ControllerMode getControllerMode() {
        String functionName = "getControllerMode";

        return CONTROLLER_MODE;
    }

    public void setControllerMode(ControllerMode controllerMode) {
        String functionName = "setControllerMode";

        CONTROLLER_MODE = controllerMode;
    }

    // Get the global scale value
    public ScaleMode getScaleMode() {
        String functionName = "getScaleMode";

        return SCALE_MODE;

    }

    // Set a global scaling mode
    public void setScaleMode(ScaleMode sm) {
        String functionName = "setScaleMode";
        SCALE_MODE = sm;
    }

    // Scale a double value based on the default scaling mode
    public double scaleValue(double value) {
        String functionName = "scaleValue";
        return scaleValue(value, SCALE_MODE);
    }

    // Scale a value based upon an exponent
    public double scaleValue(double value, ScaleMode sm) {
        String functionName = "scaleValue";
        int sign = 1;
        double exponent = 1;

        if (value < 0) {
            sign = -1;
        }
        value = abs(value);

        if (sm == ScaleMode.LINEAR) {
            return value;
        }
        else if (sm == ScaleMode.ONE_PT_FIVE) {
            exponent = 1.5;
        }
        else if (sm == ScaleMode.ONE_PT_SEVEN) {
            exponent = 1.7;
        }
        else if (sm == ScaleMode.SQUARE) {
            exponent = 2.0;
        }
        else if (sm == ScaleMode.CUBE) {
            exponent = 3.0;
        }

        return sign * (Math.pow( abs(value) , exponent));  // Retain the sign if originally negative
    }

    public double getMotorPositionInDegrees(DcMotor motor) {
        // TODO: NOT TESTED

        String functionName = "getMotorPositionInDegrees";

        if (motor != null) {
            return (motor.getCurrentPosition() * COUNTS_PER_MOTOR_REV) % 360.0;
        }

        return 0.0;
    }

    // get the drive train
    public DriveTrain getDriveTrain() {
        String functionName = "getDriveTrain";

        return DRIVE_TRAIN;
    }

    // Set the drive train
    public void setDriveTrain(DriveTrain dt) {
        String functionName = "setDriveTrain";

        DRIVE_TRAIN = dt;
    }

    public double getConfigurationPotRawValue() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotRawValue";

        if (configurationPot != null) {
            return configurationPot.getVoltage();
        }
        return 0.0;
    }

    public int getConfigurationPotCurrentDivision() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotCurrentDivision";

        if (configurationPot != null) {
            return (int) ((configurationPot.getVoltage() / configurationPot.getMaxVoltage()) * POT_DIVISIONS);
        }
        return 0;
    }

    public void getConfigurationPotBits() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotBits";

        int currDiv = getConfigurationPotCurrentDivision();
        for (int i = 0; i < potBitValues.length; i++) {
            potBitValues[i] = (currDiv & (1 << i)) != 0;
        }
    }

    // get the global debug mode
    public boolean getDebugMode() {
        String functionName = "getDebugMode";

        return DEBUG_MODE;
    }

    // Turn on debug mode for more verbose logging
    public void setDebugMode(boolean debugging) {
        String functionName = "setDebugMode";

        DEBUG_MODE = debugging;
    }

    // Determine whether or not the detected color matches our alliance for a specific color sensor
    // Use this when multiple color sensors are installed
    public boolean colorMatchesAlliance (ColorSensor colorSensor) {
        // TODO: NOT TESTED

        String functionName = "colorMatchesAlliance";
        int blueval;
        int redval;

        blueval = colorSensor.blue();
        redval = colorSensor.red();

        if ((blueval > 0) && (redval == 0)) {
            // is blue
            return alliance == Alliance.BLUE;
        }
        else {
            // is red
            if ((redval > 0) && (blueval == 0)) {
                return alliance == Alliance.RED;
            }
        }
        return true;
    }


    // Determine whether or not the detected color matches our alliance
    // Assumes default color sensor "color"
    public boolean colorMatchesAlliance () {
        // TODO: NOT TESTED

        String functionName = "colorMatchesAlliance";
        int blueval;
        int redval;

        blueval = color.blue();
        redval = color.red();

        if ((blueval > 0) && (redval == 0)) {
            // is blue
            return alliance == Alliance.BLUE;
        }
        else {
            // is red
            if ((redval > 0) && (blueval == 0)) {
                return alliance == Alliance.RED;
            }
        }
        return true;
    }

    // Turn on/off the blue and red LEDs on the CDIM
    public void setCdimLEDs(boolean blue, boolean red) {
        // TODO: NOT TESTED

        String functionName = "setCdimLEDs";
        if (cdim != null) {
            cdim.setLED(BLUE_LED, blue);
            cdim.setLED(RED_LED, red);
        }
    }

    // Set the CDIM LEDs to match the selected alliance
    public void setCdimLedsForAlliance() {
        // TODO: NOT TESTED

        String functionName = "setCdimLedsForAlliance";

        if (cdim != null) {
            if (alliance == Alliance.BLUE) {
                setCdimLEDs(true, false);
            }
            else {
                setCdimLEDs(false, true);
            }
        }
    }

    // Get the temperature reading from the IMU
    public Temperature getTemperatureFromIMU() {
        // TODO: NOT TESTED
        String functionName = "getTemperatureFromIMU";

        if (imu != null) {
            return imu.getTemperature();
        }
        return new Temperature(TempUnit.FARENHEIT, 0, 0);
    }

    // Zero out the Gyro heading
    public void zeroGyro() {
        // TODO: NOT TESTED
        String functionName = "zeroGyro";

        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }
    }

    // Test to see if an IR Beam Break sensor is broken (value low)
    public boolean isIRBeamBroken() {
        // TODO: NOT TESTED
        String functionName = "isIRBeamBroken";

        if (irBeamBreak != null) {
            return irBeamBreak.getState();
        }

        // TODO: Should throw an exception if try to use without a sensor available

        return false;
    }


    ////////
    //
    // Drive methods
    //
    ////////

    // Drive routine that supports Mecanum wheels.  Will maintain a gyro heading.
    public void driveMecanumCartesian (double x, double y, double rotation, double gyroHeading, boolean inverted) {
        String functionName = "driveMecanumCartesian";
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

        driveMecanumCartesian (x, y, rotation, 0.0, inverted);
    }

    // Mecanum drive routine that supports rotation, 0 degree gyro setting, no inversion
    public void driveMecanumCartesian (double x, double y, double rotation) {
        String functionName = "driveMecanumCartesian";

        driveMecanumCartesian (x, y, rotation, 0.0, false);
    }

    // Mecanum drive that support only x,y, no rotation, gyro heading or inversion
    public void driveMecanumCartesian (double x, double y) {
        String functionName = "driveMecanumCartesian";

        driveMecanumCartesian (x, y, 0.0, 0.0, false);
    }

    // Two wheel drive using rear wheels only
    public void driveRearTwoWheel(double LBPower, double RBPower) {
        // TODO: NOT TESTED

        String functionName = "driveRearTwoWheel";

        drive(0.0, 0.0, LBPower, RBPower);
    }

    // Two wheel drive using front wheels only
    public void driveFrontTwoWheel(double LFPower, double RFPower) {
        // TODO: NOT TESTED

        String functionName = "driveFrontTwoWheel";

        drive(LFPower, RFPower, 0.0, 0.0);
    }

    // TODO: Make this into a generic drive routine to support all different drive trains
    // Should support:
    //   Two wheel rear drive
    //   Two wheel front drive
    //   Two wheel center drive
    //   Four wheel drive (normal wheels)
    //   Mecanum drive
    //   Omniwheel drive
    //   45 Degree Four wheel Omniwheel drive
    public void drive(double LFPower, double RFPower, double LBPower, double RBPower) {
        // TODO: NOT TESTED

        String functionName = "drive";

        // Normalize the power of each motor so that the maximum motor power is MAX_MOTOR_SPEED
        double maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LBPower, RBPower)));
        if (maxPower > MAX_MOTOR_SPEED) {
            LFPower /= maxPower;
            RFPower /= maxPower;
            LBPower /= maxPower;
            RBPower /= maxPower;
        }

        // TODO: Honor the drive train.  If inputs do not match the Drive Train, throw an exception

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

    ////////
    //
    // Autonomous and automated functions
    //
    ////////

    // Use encoders to drive a specific distance
    public void driveDistanceInInches(double distance) {
        // TODO: NOT TESTED
        String functionName = "driveDistanceInInches";

        // TODO: Implement this
    }

    // Follow a line using ODS until distance to an object is at or less than a value
    public void driveFollowLineUntilDistance(double distance) {
        // TODO: NOT TESTED
        String functionName = "driveFollowLineUntilDistance";

        // TODO: Implement this
    }

    // Drive forward and push a beacon button.  Wait. Detect color.  If not alliance color, repeat.
    // Note: can be used in TeleOp to automate this
    public void pushBeaconUntilMatchesAlliance() {
        // TODO: NOT TESTED
        String functionName = "pushBeaconUntilMatchesAlliance";

        // TODO: Implement this

    }

    // Turn the robot a specific number of degrees (positive or negative) using Gyro
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    // Turns will always honor the direction, even if the turn could be optimized by turning in the
    // opposite direction.  E.g. a turn of -270 degrees will turn to the left instead of turning
    // to the right 90 degrees.
    public void turnByDegreesGyro(double degrees, double speed) {
        // TODO: NOT TESTED
        String functionName = "turnByDegreesGyro";
        double turnDegrees = 0;

        // TODO: Implement this

        // Limit turnDegrees to 360 or less

    }

    // Turn the robot a specific number of degrees (positive or negative) using Adafruit IMU
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    // Turns will always honor the direction, even if the turn could be optimized by turning in the
    // opposite direction.  E.g. a turn of -270 degrees will turn to the left instead of turning
    // to the right 90 degrees.
    public void turnByDegreesIMU(double degrees, double speed) {
        // TODO: NOT TESTED
        String functionName = "turnByDegreesIMU";
        double turnDegrees = 0;

        // TODO: Implement this

        // Limit turnDegrees to 360 or less


    }

    // Turn the robot a specific number of degrees (positive or negative)
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    // Turns will always honor the direction, even if the turn could be optimized by turning in the
    // opposite direction.  E.g. a turn of -270 degrees will turn to the left instead of turning
    // to the right 90 degrees.
    public void turnByDegrees(double degrees, double speed) throws RobotConfigurationException {
        // TODO: NOT TESTED
        String functionName = "turnByDegrees";

        // If there is a Modern Robotics Gyro, use that first.
        // If no MR Gyro, try to use an Adafruit IMU.
        if (gyro != null) {
            turnByDegreesGyro(degrees, speed);
        }
        else if (imu != null) {
            turnByDegreesIMU(degrees, speed);
        }
        else {
            // Can't turn without a gyro or IMU
            throw new RobotConfigurationException("No gyro or IMU present!");
        }

    }

    // Fire and re-arm the choo choo launcher
    //
    // Note: This can be done multiple ways:
    // 1. Use a motor with an encoder.  Motor resets to a specific angle/position.
    //    Careful calibration will be required, and adjustments to the motor, such as
    //    rotation of the motor in the mount, may require recalibration of the angle.
    //    Initial calibration could happen prior to autonomous by using alignment marks on the robot
    //    launcher to denote the appropriate starting position.  Firing the launcher, or impact with
    //    field elements may affect alignment going into TeleOp.
    // 2. Use an sensor to detect when the launcher has reached appropriate armed position.
    //    Care has to be taken to ensure that sensors are mounted appropriately and that the
    //    chosen sensor has a response speed that is sufficient to detect that the armed position
    //    has been reached in order to stop in time.  Firing the launcher must detect both a
    //    "not armed" and an "armed" value in order to prevent falsely detecting that the launcher
    //    has returned to an armed position because it is still within the sensor's defined
    //    zone for detection.
    //
    public void fireAndArmChooChooLauncher() {
        // TODO: NOT TESTED
        String functionName = "fireAndArmChooChooLauncher";

        // TODO: Implement this

    }

    ////////
    //
    // Initialization
    //
    ////////

    // Initialize motors
    private void initializeMotors() throws RobotConfigurationException {
        String functionName = "initializeMotors";

        // Initialize the motor metadata map
        // TODO: is there a better place to put this initialization?
        motorMetadataMap.initialize();

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }

        // Define Motors
//        LFMotor   = hwMap.dcMotor.get("LFMotor");
//        RFMotor  = hwMap.dcMotor.get("RFMotor");
        LBMotor   = hwMap.dcMotor.get("LBMotor");
        RBMotor  = hwMap.dcMotor.get("RBMotor");
        ChooChooMotor = hwMap.dcMotor.get("ChooChooMotor");
        BeaterMotor = hwMap.dcMotor.get("BeaterMotor");

        // Map the motors to their metadata, which includes encoder counts, etc.
//        motorData.put(LFMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(RFMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(LBMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(RBMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(ChooChooMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.TETRIX));
        motorData.put(BeaterMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.TETRIX));


        // Set starting motor directions
//        LFMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        RFMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LBMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RBMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        ChooChooMotor.setDirection(DcMotor.Direction.REVERSE);
        BeaterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ChooChooMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BeaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set braking mode for each motor
        // SDK default for year 2 is to put the motor in BRAKE mode.  Year 1 was FLOAT.
//        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ChooChooMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BeaterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
//        LFMotor.setPower(0.0);
//        RFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RBMotor.setPower(0.0);
        ChooChooMotor.setPower(0.0);
        BeaterMotor.setPower(0.0);

        // Set motor max speed based on encoder counts from motor metadata
        // Count per revolution is dependent on the motor type
        // If not set, defaults to Tetrix  (1440) which impacts performance of AndyMark motors...
//        LFMotor.setMaxSpeed((int)motorData.get(LFMotor).encoderCountPerRevolution);
//        RFMotor.setMaxSpeed((int)motorData.get(RFMotor).encoderCountPerRevolution);
        LBMotor.setMaxSpeed((int)motorData.get(LBMotor).encoderCountPerRevolution);
        RBMotor.setMaxSpeed((int)motorData.get(RBMotor).encoderCountPerRevolution);
        ChooChooMotor.setMaxSpeed((int)motorData.get(ChooChooMotor).encoderCountPerRevolution);
        BeaterMotor.setMaxSpeed((int)motorData.get(BeaterMotor).encoderCountPerRevolution);
    }

    // Initialize servos
    private void initializeServos() throws RobotConfigurationException {
        String functionName = "initializeServos";

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }
    }

    // Initialize sensors
    private void initializeSensors() throws RobotConfigurationException {
        String functionName = "initializeSensors";

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }

        // Get Sensors from Hardware Map
        cdim = hwMap.deviceInterfaceModule.get("cdim");
//        gyro = hwMap.gyroSensor.get("gyro");
        imu = hwMap.get(BNO055IMU.class, "imu");
        color = hwMap.colorSensor.get("color");
        afcolor = hwMap.colorSensor.get("afcolor");
        ods = hwMap.opticalDistanceSensor.get("ods");
        allianceChannel = hwMap.digitalChannel.get("alliance");
        startPositionChannel = hwMap.digitalChannel.get("startposition");
        strategyChannel = hwMap.digitalChannel.get("strategy");
        irBeamBreak = hwMap.digitalChannel.get("irBeamBreak");


        ////////
        //
        // I2C Sensors
        //
        ////////

        // Initialize Gyro if present
        if (gyro != null) {
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        //
        // Adafruit IMU
        //
        if (imu != null) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT; // Set temperature unit.

            // Calibration data file is found in FIRST/settings/<data file>
            // Calibration data increases the accuracy of the IMU and reduces the time needed for initialization.
            parameters.calibrationDataFile = "MentorAdafruitIMUCalibration.json"; // see the calibration sample opmode
            imu.initialize(parameters);
        }

        //
        // Modern Robotics Color Sensor
        //
        // Turn off the LED on the color sensor by default
        // LED light is not needed for detecting the beacons since they are a light source.
        // If detecting color of an object that is not a light source, turn on the LED.
        if (color != null) {
            color.enableLed(false);
        }

        //
        // Adafruit Color Sensor
        //
        if (afcolor != null) {
            if (cdim != null) {
                // Connect pin for the LED to a digital channel
                cdim.setDigitalChannelState(ADAFRUIT_COLOR_LED_CHANNEL, false);
            }
        }


        ////////
        //
        // Analog Sensors
        //
        ////////

        //
        // ODS
        //
        // Turn on the LED on the ODS
        if (ods != null) {
            ods.enableLed(true);
        }


        ////////
        //
        // Digital Channels
        //
        ////////

        //
        // Alliance configuration switch
        //
        if (allianceChannel != null) {
            allianceChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (allianceChannel.getState()) {
                alliance = Alliance.BLUE;
                setCdimLEDs(true, false);
            } else {
                alliance = Alliance.RED;
                setCdimLEDs(false, true);
            }
        }
        else {
            alliance = Alliance.BLUE;
        }

        //
        // Start Position configuration switch
        //
        if (startPositionChannel != null) {
            startPositionChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (startPositionChannel.getState()) {
                startPosition = StartPosition.POSITION1;
            } else {
                startPosition = StartPosition.POSITION2;
            }
        }
        else {
            startPosition = StartPosition.POSITION1;
        }

        //
        // Strategy configuration switch
        //
        if (strategyChannel != null) {
            strategyChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (strategyChannel.getState()) {
                strategy = Strategy.STRATEGY1;
            } else {
                strategy = Strategy.STRATEGY2;
            }
        }
        else {
            strategy = Strategy.STRATEGY1;
        }

        //
        // Configuration Potentiometer
        //
        if (configurationPot != null) {
            // TODO: Add code to read potentiometer if used.
        }

    }

    private void getRobotConfigurationDataFromFile() {
        // TODO: NOT TESTED
        String functionName = "getRobotConfigurationDataFromFile";

        // TODO: Implement this
        try {
            //File file = hwMap.appContext // AppUtil.getInstance().getSettingsFile();
            File file = null;
            String serialized = ReadWriteFile.readFileOrThrow(file);
            robotConfigurationData = MentorHardwareRobotConfiguration.deserialize(serialized);
            //writeCalibrationData(data);
        }
        catch (IOException e)
        {
            // Ignore the absence of the indicated file, etc
        }
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        String functionName = "init";

        setLoggingMode(LoggingMode.NONE);

        // Save reference to Hardware map
        hwMap = ahwMap;

        try {
            initializeMotors();
        }
        catch (RobotConfigurationException rce) {
            // TODO: Take action
        }

        try {
            initializeServos();
        }
        catch (RobotConfigurationException rce) {
            // TODO: Take action
        }

        try {
            initializeSensors();
        }
        catch (RobotConfigurationException rce) {
            // TODO: Take action
        }

    }

//    /***
//     *
//     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//     * periodic tick.  This is used to compensate for varying processing times for each cycle.
//     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//     *
//     * @param periodMs  Length of wait cycle in mSec.
//     */
//    public void waitForTick(long periodMs) {
//
//        // TODO: NOT TESTED
//
//        long  remaining = periodMs - (long)runtime.milliseconds();
//
//        // sleep for the remaining portion of the regular cycle period.
//        if (remaining > 0) {
//            try {
//                sleep(remaining);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
//        }
//
//        // Reset the cycle clock for the next pass.
//        runtime.reset();
//    }
}

