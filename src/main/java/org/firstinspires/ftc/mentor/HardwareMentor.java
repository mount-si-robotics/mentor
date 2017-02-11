package org.firstinspires.ftc.mentor;

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.mentor.common.Alliance;
import org.firstinspires.ftc.mentor.common.ControllerMode;
import org.firstinspires.ftc.mentor.common.DriveTrain;
import org.firstinspires.ftc.mentor.common.LoggingMode;
import org.firstinspires.ftc.mentor.common.MentorHardwareRobotConfiguration;
import org.firstinspires.ftc.mentor.common.MotorMetadata;
import org.firstinspires.ftc.mentor.common.MotorMetadataMap;
import org.firstinspires.ftc.mentor.common.RobotConfigurationException;
import org.firstinspires.ftc.mentor.common.ScaleMode;
import org.firstinspires.ftc.mentor.common.StartPosition;
import org.firstinspires.ftc.mentor.common.Strategy;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.nextAfter;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.Range;
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
 * Motor channel:  Left middle drive motor:     "LMMotor"
 * Motor channel:  Right middle drive motor:    "RMMotor"
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
 *  Add support for a configuration file to allow dynamic changes to values instead of hardcoding
 *
 */

public class HardwareMentor
{
    // Class name for logging purposes
    private String className = "HardwareMentor";

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode LINEAR_OPMODE = null;

    private static final double MM_TO_INCHES = 25.4;

    // Maximum number of motors allowed by rules
    private static final int MAX_MOTORS_ALLOWED = 8;

    /* Public OpMode members. */
    private boolean DEBUG_MODE = false;  // Debugging is off by default

    private DcMotor  LFMotor   = null; // Left Front drive
    private DcMotor  RFMotor  = null;  // Right Front drive
    private DcMotor  LMMotor = null; // Left Middle drive
    private DcMotor  RMMotor = null; // Right Middle drive
    private DcMotor  LBMotor   = null; // Left Back drive
    private DcMotor  RBMotor  = null;  // Right Back drive
    private DcMotor  ChooChooMotor = null;  // Launcher motor
    private DcMotor  BeaterMotor = null;    // Ball gather motor
//    private DcMotor  LiftMotor1 = null;  // Lift motor
//    private DcMotor  LiftMotor2 = null;  // Lift motor 2
//    private DcMotor  SlideMotor = null; // LinearSlide motor

    // Maps to contain the motors allocated on this robot.
    // Add motors to the map(s) during init
    private Map<DcMotor, String> allMotorMap = new HashMap<>();
    private Map<DcMotor, String> driveMotorMap = new HashMap<>();

    private MotorMetadataMap motorMetadataMap = new MotorMetadataMap();

    // Mapping occurs in initializeMotors()
    private Map<DcMotor, MotorMetadata> motorData = new HashMap<>();

    private float DEFAULT_DEADZONE = 0.05f;
    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    //double APPROACH_SPEED = 0.5; // drive speed
    private double DEFAULT_DRIVE_SPEED = 0.5; // drive speed to use when not specified
    private double DEFAULT_TURN_SPEED = 0.5;  // turn speed to use when not specified
    private double DRIVE_WHEEL_DIAMETER_INCHES = 3.0;
    private boolean isSlowDrive = false;
    private double slowDriveDivisor = 2; // Divide speed by this number if in slow speed mode.
                                         // Dividing by 2 means speed is half the max speed

    // Motor Constants
    static final double MIN_MOTOR_SPEED = -1.0;
    private static final double MAX_MOTOR_SPEED = 1.0;

    // Default rotation speed and direction for the choo choo launcher motor
    private double CHOO_CHOO_MOTOR_SPEED = 0.5;
    private double CHOO_CHOO_MOTOR_GEAR_REDUCTION = 1.0;

    // Default rotation speed and direction for the beater (ball collector) motor
    private double BEATER_MOTOR_SPEED = 0.8;
    boolean isBeaterRunning = false;

    private static final double DRIVE_GEAR_REDUCTION = (1/3);     // This is < 1.0 if geared UP. 120 tooth on motor to 40 on wheel axle = 0.33
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//    double ENCODER_DEGREE_PER_COUNT = 360.0 / COUNTS_PER_MOTOR_REV;
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;

    private static final int ONE_SECOND_IN_MS = 1000;
    private double RETREAT_DISTANCE_INCHES = -2.0;

    private DeviceInterfaceModule cdim = null;
    public GyroSensor gyro = null;  // Modern Robotics Gyroscope
    public ColorSensor color = null; // Modern Robotics Color
    private ColorSensor afcolor = null; // Adafruit Color Sensor

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    private static final int ADAFRUIT_COLOR_LED_CHANNEL = 5;

    private OpticalDistanceSensor ods = null;  // Modern Robotics ODS
    public BNO055IMU imu;  // Adafruit IMU

    // Hardware switches for selecting runtime parameters
    private DigitalChannel allianceChannel = null;
    private DigitalChannel startPositionChannel = null;
    private DigitalChannel strategyChannel = null;

    // IR Beam Break Digital Sensor
    private DigitalChannel irBeamBreak = null;

    // Potentiometer for providing analog input
    private AnalogInput configurationPot = null;

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

    // Sane defaults
    private DriveTrain DRIVE_TRAIN = DriveTrain.NONE;
    private ScaleMode SCALE_MODE = ScaleMode.LINEAR;
    public ControllerMode CONTROLLER_MODE = ControllerMode.TANK;
    private LoggingMode LOGGING_MODE = LoggingMode.NONE;

    // TODO: better name for log file
    private String LOGGING_FILE = "logfile";

    private static final int BLUE_LED    = 0;     // Blue LED channel on DIM
    private static final int RED_LED     = 1;     // Red LED Channel on DIM

    private Alliance alliance = null;
    private Strategy strategy = null;
    private StartPosition startPosition = null;

    // Configuration file for overriding default settings
    private MentorHardwareRobotConfiguration robotConfigurationData = null;
    private String robotConfigurationDataFile = null;

    /*
     * Robot components
     */
//    LiftMechanism lift = null;
//    LinearSlide slide = null;


    ///////////////////////////////////////////////////
    ///////////////////////////////////////////////////

    /* Constructor */
    HardwareMentor(){
        String functionName = "HardwareMentor";
        // Empty constructor
        // Nothing to see here...
    }

    // Get a handle to the OpMode that is using this class
    private void setLinearOpMode(LinearOpMode opMode) {
        String functionName = "setLinearOpMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        LINEAR_OPMODE = opMode;
    }

    // Get the currently set LoggingMode
    LoggingMode getLoggingMode() {
        String functionName = "getLoggingMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return LOGGING_MODE;
    }

    void setLoggingMode(LoggingMode loggingMode) {
        // TODO: NOT TESTED YET
        String functionName = "setLoggingMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

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

    // Set the game pad dead zone
    void setGamepadDeadzone(float deadzone) {
        String functionName = "setGamepadDeadzone";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (LINEAR_OPMODE != null) {
            LINEAR_OPMODE.gamepad1.setJoystickDeadzone(deadzone);
            LINEAR_OPMODE.gamepad2.setJoystickDeadzone(deadzone);
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
            DbgLog.msg("%s", functionName);
        }

        setGamepadDeadzone(DEFAULT_DEADZONE);
    }

    ControllerMode getControllerMode() {
        String functionName = "getControllerMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return CONTROLLER_MODE;
    }

    void setControllerMode(ControllerMode controllerMode) {
        String functionName = "setControllerMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        CONTROLLER_MODE = controllerMode;
    }

    // Get the global scale value
    ScaleMode getScaleMode() {
        String functionName = "getScaleMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return SCALE_MODE;

    }

    // Set a global scaling mode
    void setScaleMode(ScaleMode sm) {
        String functionName = "setScaleMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        SCALE_MODE = sm;
    }

    // Scale a double value based on the default scaling mode
    public double scaleValue(double value) {
        String functionName = "scaleValue";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return scaleValue(value, SCALE_MODE);
    }

    // Scale a value based upon an exponent
    double scaleValue(double value, ScaleMode sm) {
        String functionName = "scaleValue";
        int sign;
        double exponent = 1;

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        sign = (value < 0) ? -1 : 1;

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

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (motor != null) {
            return (motor.getCurrentPosition() * motorData.get(motor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) % 360.0;
        }

        return 0.0;
    }

    // get the drive train
    DriveTrain getDriveTrain() {
        String functionName = "getDriveTrain";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return DRIVE_TRAIN;
    }

    // Set the drive train
    void setDriveTrain(DriveTrain dt) {
        String functionName = "setDriveTrain";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        DRIVE_TRAIN = dt;
    }

    public double getConfigurationPotRawValue() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotRawValue";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (configurationPot != null) {
            return configurationPot.getVoltage();
        }
        return 0.0;
    }

    private int getConfigurationPotCurrentDivision() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotCurrentDivision";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (configurationPot != null) {
            return (int) ((configurationPot.getVoltage() / configurationPot.getMaxVoltage()) * POT_DIVISIONS);
        }
        return 0;
    }

    public void getConfigurationPotBits() {
        // TODO: NOT TESTED

        String functionName = "getConfigurationPotBits";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        int currDiv = getConfigurationPotCurrentDivision();
        for (int i = 0; i < potBitValues.length; i++) {
            potBitValues[i] = (currDiv & (1 << i)) != 0;
        }
    }

    // get the global debug mode
    boolean getDebugMode() {
        String functionName = "getDebugMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return DEBUG_MODE;
    }

    // Turn on debug mode for more verbose logging
    void setDebugMode(boolean debugging) {
        String functionName = "setDebugMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        DEBUG_MODE = debugging;
    }

    // Determine whether or not the detected color matches our alliance for a specific color sensor
    // Use this when multiple color sensors are installed
    public boolean colorMatchesAlliance (ColorSensor colorSensor) {
        // TODO: NOT TESTED

        String functionName = "colorMatchesAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

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
    boolean colorMatchesAlliance() {
        // TODO: NOT TESTED
        String functionName = "colorMatchesAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        int blueval;
        int redval;

        if (color != null) {
            blueval = color.blue();
            redval = color.red();
        }
        else {
            if (afcolor != null) {
                blueval = afcolor.blue();
                redval = afcolor.red();
            } else {
                try {
                    throw (new RobotConfigurationException(functionName + ": No color sensors configured."));
                } catch (RobotConfigurationException e) {
                    e.printStackTrace();
                }
                return true;
            }
        }

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
    private void setCdimLEDs(boolean blue, boolean red) {
        String functionName = "setCdimLEDs";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (cdim != null) {
            cdim.setLED(BLUE_LED, blue);
            cdim.setLED(RED_LED, red);
        }
    }

    // Read alliance digital switch and set alliance
    private void setAllianceFromSwitch() {
        String functionName = "setAllianceFromSwitch";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // By default, set alliance to BLUE
        alliance = Alliance.BLUE;

        if (allianceChannel != null) {
            allianceChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (allianceChannel.getState()) {
                setAlliance(Alliance.BLUE);
            } else {
                setAlliance(Alliance.RED);
            }
        }
    }

    // Set the CDIM LEDs to match the selected alliance
    public void setCdimLedsForAlliance() {
        String functionName = "setCdimLedsForAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (cdim != null) {
            if (alliance == Alliance.BLUE) {
                setCdimLEDs(true, false);
            }
            else {
                setCdimLEDs(false, true);
            }
        }
    }

    // Get alliance
    public Alliance getAlliance() {
        String functionName = "getAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return alliance;
    }

    // Set alliance
    void setAlliance(Alliance a) {
        String functionName = "setAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        alliance = a;
    }

    // Get start position
    public StartPosition getStartPosition() {
        String functionName = "getStartPosition";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return startPosition;
    }

    // Set start position
    void setStartPosition(StartPosition s) {
        String functionName = "setStartPosition";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        startPosition = s;
    }

    // Get strategy
    public Strategy getStrategy() {
        String functionName = "getStrategy";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return strategy;
    }

    // Set strategy
    void setStrategy(Strategy s) {
        String functionName = "setStrategy";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        strategy = s;
    }

    // Get the temperature reading from the IMU
    public Temperature getTemperatureFromIMU() {
        // TODO: NOT TESTED
        String functionName = "getTemperatureFromIMU";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (imu != null) {
            return imu.getTemperature();
        }

        return new Temperature(TempUnit.FARENHEIT, 0, 0);
    }

    // Zero out the Gyro heading
    void zeroGyro() {
        // TODO: NOT TESTED
        String functionName = "zeroGyro";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }
    }

    // Test to see if an IR Beam Break sensor is broken (value low)
    public boolean isIRBeamBroken() {
        // TODO: NOT TESTED
        String functionName = "isIRBeamBroken";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (irBeamBreak != null) {
            return irBeamBreak.getState();
        }

        // Throw an exception if try to use without a sensor available
        try {
            throw(new RobotConfigurationException(functionName + ": No beam break sensor configured!"));
        } catch (RobotConfigurationException e) {
            e.printStackTrace();
        }

        return false;
    }

    // Play a sound stored in a resource file
    public void playSound(int sound) {
        // TODO: NOT TESTED
        String functionName = "playSound";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        MediaPlayer mediaPlayer = MediaPlayer.create(hwMap.appContext, sound);
        mediaPlayer.start();
    }

    public boolean isSlowDrive() {
        String functionName = "isSlowDrive";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return isSlowDrive;
    }

    public void setSlowDrive(boolean slow) {
        // TODO: NOT TESTED
        String functionName = "setSlowDrive";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (slow) {
            isSlowDrive = true;
        }
        else {
            isSlowDrive = false;
        }
    }

    public double getSlowDriveDivisor() {
        // TODO: NOT TESTED
        String functionName = "getSlowDriveDivisor";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        return slowDriveDivisor;
    }

    public void setBallLoaderBlockPosition() {
        // TODO: NOT TESTED
        String functionName = "setBallLoaderBlockPosition";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // TODO: NOT IMPLEMENTED
        // Set Servo to position to block the ball

    }

    public void setBallLoaderOpenPosition() {
        // TODO: NOT TESTED
        String functionName = "setBallLoaderOpenPosition";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // TODO: NOT IMPLEMENTED YET
        // Set Servo to position to allow ball to move into launcher
    }

    public void loadBall() {
        // TODO: NOT TESTED
        String functionName = "loadBall";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // TODO: NOT IMPLEMENTED YET

        // Open the servo
        setBallLoaderOpenPosition();

        // Wait
        LINEAR_OPMODE.sleep(500);

        // Close the servo
        setBallLoaderBlockPosition();
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
            DbgLog.msg("%s", functionName);
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
            DbgLog.msg("%s", functionName);
        }

        driveMecanumCartesian (x, y, rotation, 0.0, inverted);
    }

    // Mecanum drive routine that supports rotation, 0 degree gyro setting, no inversion
    public void driveMecanumCartesian (double x, double y, double rotation) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        driveMecanumCartesian (x, y, rotation, 0.0, false);
    }

    // Mecanum drive that support only x,y, no rotation, gyro heading or inversion
    public void driveMecanumCartesian (double x, double y) {
        String functionName = "driveMecanumCartesian";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        driveMecanumCartesian (x, y, 0.0, 0.0, false);
    }

    // Two wheel drive using rear wheels only
    public void driveRearTwoWheel(double LBPower, double RBPower) {
        // TODO: NOT TESTED

        String functionName = "driveRearTwoWheel";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        drive(0.0, 0.0, 0.0, 0.0, LBPower, RBPower);
    }

    // Two wheel drive using front wheels only
    public void driveFrontTwoWheel(double LFPower, double RFPower) {
        // TODO: NOT TESTED

        String functionName = "driveFrontTwoWheel";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        drive(LFPower, RFPower, 0.0, 0.0, 0.0, 0.0);
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
    public void drive(double LFPower, double RFPower, double LMPower, double RMPower, double LBPower, double RBPower) {
        // TODO: NOT TESTED

        String functionName = "drive";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // Normalize the power of each motor so that the maximum motor power is MAX_MOTOR_SPEED
        double maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LMPower, Math.max(RMPower, Math.max(LBPower, RBPower)))));
        if (maxPower > MAX_MOTOR_SPEED) {
            LFPower /= maxPower;
            RFPower /= maxPower;
            LMPower /= maxPower;
            RMPower /= maxPower;
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
        if (LMMotor != null) {
            LMMotor.setPower(LMPower);
        }
        if (RMMotor != null) {
            RMMotor.setPower(RMPower);
        }
        if (LBMotor != null) {
            LBMotor.setPower(LBPower);
        }
        if (RBMotor != null) {
            RBMotor.setPower(RBPower);
        }
    }

    // drive method specifying only left/right values
    public void drive (double leftPower, double rightPower) {
        // TODO: NOT TESTED
        String functionName = "drive";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // Use same speed for all potential left/right motors
        drive(leftPower, rightPower, leftPower, rightPower, leftPower, rightPower);
    }

    // Stop drive motors
    void stopDriveMotors() {
        String functionName = "stopDriveMotors";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setPower(0.0);
        }
    }

    // Stop drive motors and wait
    public void stopDriveMotors(long delayMillis) {
        // TODO: NOT TESTED
        String functionName = "stopDriveMotors";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        stopDriveMotors();

        try {
            LINEAR_OPMODE.wait(delayMillis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Start beater motor
    void startBeaterMotor() {
        String functionName = "startBeaterMotor";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (BeaterMotor != null) {
            BeaterMotor.setPower(BEATER_MOTOR_SPEED);

            isBeaterRunning = true;
        }
    }

    void reverseBeaterMotor() {
        String functionName = "reverseBeaterMotor";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        double power;

        if (BeaterMotor != null) {
            power = BeaterMotor.getPower();

            stopBeaterMotor();

            BeaterMotor.setPower(-1 * power);
            isBeaterRunning = true;
        }
    }

    // Stop beater motor
    void stopBeaterMotor() {
        String functionName = "stopBeaterMotor";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (BeaterMotor != null) {
            BeaterMotor.setPower(0.0);
        }
        isBeaterRunning = false;
    }

    // Stop ChooChoo Motor
    void stopChooChooMotor() {
        String functionName = "stopChooChooMotor";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (ChooChooMotor != null) {
            ChooChooMotor.setPower(0.0);
        }
    }

    ////////
    //
    // Autonomous and automated functions
    //
    ////////


    // Set the mode for the drive motors
    void setDriveMotorMode(DcMotor.RunMode runMode) {
        // TODO: NOT TESTED
        String functionName = "setDriveMotorMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setMode(runMode);
        }
    }

    // Get the drive motor run mode
    DcMotor.RunMode getDriveMotorMode() {
        // TODO: NOT TESTED
        String functionName = "getDriveMotorMode";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            return motor.getMode();
        }

        // Should never reach this point unless no drive motors are configured.
        try {
            throw(new RobotConfigurationException(functionName + ": No drive motors configured."));
        } catch (RobotConfigurationException e) {
            e.printStackTrace();
        }

        return null;
    }

    // Set the zero power behavior for drive motors
    private void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        String functionName = "setDriveMotorZeroPowerBehavior";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        for (DcMotor motor : driveMotorMap.keySet()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // Check to see if drive motors are busy
    private boolean driveMotorsBusy() {
        String functionName = "driveMotorsBusy";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        boolean returnVal = false;

        for (DcMotor motor : driveMotorMap.keySet()) {
            if (motor.isBusy()) {
                returnVal |= true;
            }
        }

        return returnVal;
    }

    // Use encoders to drive a specific distance
    void driveDistanceInInches(double left_distance, double right_distance, double speed) {
        String functionName = "driveDistanceInInches";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        int newLeftTarget;
        int newRightTarget;

        // If this is not a LinearOpMode we can't proceed
        if (LINEAR_OPMODE == null) {
            try {
                throw(new RobotConfigurationException(functionName + ": LINEAR_OPMODE is null"));
            } catch (RobotConfigurationException e) {
                e.printStackTrace();
            }
            return;
        }

        // Get the current run mode and save it for restoring it later
        DcMotor.RunMode runMode = getDriveMotorMode();

        // Set to RUN_TO_POSITION
        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        if (LFMotor != null) {
            newLeftTarget = LFMotor.getCurrentPosition() + (int) ((left_distance * motorData.get(LFMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            LFMotor.setTargetPosition(newLeftTarget);
        }
        if (LMMotor != null) {
            newLeftTarget = LMMotor.getCurrentPosition() + (int) ((left_distance * (int)motorData.get(LMMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            LMMotor.setTargetPosition(newLeftTarget);
        }
        if (LBMotor != null) {
            newLeftTarget = LBMotor.getCurrentPosition() + (int) ((left_distance * (int)motorData.get(LBMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            LBMotor.setTargetPosition(newLeftTarget);
        }
        if (RFMotor != null) {
            newRightTarget = RFMotor.getCurrentPosition() + (int) ((right_distance * (int)motorData.get(RFMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            RFMotor.setTargetPosition(newRightTarget);
        }
        if (RMMotor != null) {
            newRightTarget = RMMotor.getCurrentPosition() + (int) ((right_distance * (int)motorData.get(RMMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            RMMotor.setTargetPosition(newRightTarget);
        }
        if (RBMotor != null) {
            newRightTarget = RBMotor.getCurrentPosition() + (int) ((right_distance * (int)motorData.get(RBMotor).encoderCountPerRevolution * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_INCHES * PI));
            RBMotor.setTargetPosition(newRightTarget);
        }

        drive(speed, speed, speed, speed, speed, speed);

        while (driveMotorsBusy() && LINEAR_OPMODE.opModeIsActive()) {
            LINEAR_OPMODE.telemetry.addData(functionName, ": driving...");
            LINEAR_OPMODE.telemetry.update();
        }

        stopDriveMotors();
        // TODO: Should we wait here for the motors to stop?
//        stopDriveMotors(250); // Stop motors and delay 250 milliseconds

        // Restore the original run mode
        setDriveMotorMode(runMode);

    }

    // Use encoders to drive a specific distance using default speed
    void driveDistanceInInches(double left_distance, double right_distance) {
        String functionName = "driveDistanceInInches";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        driveDistanceInInches(left_distance, right_distance, DEFAULT_DRIVE_SPEED);

    }

    // Use encoders to drive a specific distance
    void driveDistanceInMM(double left_distance, double right_distance, double speed) {
        String functionName = "driveDistanceInMM";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        driveDistanceInInches(left_distance * MM_TO_INCHES, right_distance * MM_TO_INCHES, speed * MM_TO_INCHES);
    }

    // Use encoders to drive a specific distance using default speed
    void driveDistanceInMM(double left_distance, double right_distance) {
        String functionName = "driveDistanceInMM";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        driveDistanceInMM(left_distance, right_distance, DEFAULT_DRIVE_SPEED);
    }

    // Drive until meet a white line on the floor
    void driveUntilLineDetected() {
        // TODO: NOT TESTED
        String functionName = "driveUntilLineDetected";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        double lightThreshold = 0.2;
        double timeout = 5.0;
        double startTime = runtime.seconds();

        while (LINEAR_OPMODE.opModeIsActive() && (ods.getRawLightDetected() < lightThreshold) && ((runtime.seconds() - startTime) < timeout)) {
            drive(DEFAULT_DRIVE_SPEED, DEFAULT_DRIVE_SPEED);
        }

        stopDriveMotors();

    }

    // Follow a line using ODS until distance to an object is at or less than a value
    void driveFollowLineUntilDistance(double distance) {
        // TODO: NOT TESTED
        String functionName = "driveFollowLineUntilDistance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        double lightThreshold = 0.2;

        // TODO: IMPLEMENT THIS
//        while (LINEAR_OPMODE.opModeIsActive() && ods.getRawLightDetected() < lightThreshold) {
//            drive(DEFAULT_DRIVE_SPEED, DEFAULT_DRIVE_SPEED);
//        }

        stopDriveMotors();
    }

    // Drive forward and push a beacon button.  Wait. Detect color.  If not alliance color, repeat.
    // Note: can be used in TeleOp to automate this
    void pushBeaconUntilMatchesAlliance() {
        // TODO: NOT TESTED
        String functionName = "pushBeaconUntilMatchesAlliance";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // TODO: Implement this

        do {
            // TODO: Drive forward and push beacon

            // TODO: Use touch sensor to detect beacon pressed?

            // Wait 2 seconds for beacon to change
            LINEAR_OPMODE.sleep(2 * ONE_SECOND_IN_MS);

            // Check beacon color
            if (!colorMatchesAlliance()) {
                driveDistanceInInches(RETREAT_DISTANCE_INCHES, RETREAT_DISTANCE_INCHES);

                // If not alliance color, backup and wait 3 more seconds
                LINEAR_OPMODE.sleep(3 * ONE_SECOND_IN_MS);
            }
        } while (! colorMatchesAlliance());

        // Repeat until alliance color match


    }

    // Turn the robot a specific number of degrees (positive or negative) using Gyro
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    private void turnByDegreesGyro(double degrees, double speed) {
        // TODO: NOT TESTED
        String functionName = "turnByDegreesGyro";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        double gain = 0.7; // Tune this value as needed +/- for accurate turns
        double currentHeading;
        double timeout = 10.0;  // TODO: what is appropriate timeout value for a turn?
        double headingError;
        double driveSteering;
        double leftPower;
        double rightPower;

        // Limit turnDegrees to 360 or less

        runtime.reset();
        do {
            currentHeading = gyro.getHeading();

            // Optimize turn direction
            if (currentHeading > 180.0) {
                currentHeading -= 360.0;
            }

            headingError = currentHeading - degrees;
            driveSteering = headingError * gain;

            leftPower = DEFAULT_TURN_SPEED + driveSteering;
            rightPower = DEFAULT_TURN_SPEED - driveSteering;

            leftPower = Range.clip(leftPower, 0.0, 1.0);
            rightPower = Range.clip(rightPower, 0.0, 1.0);

            drive(leftPower,rightPower);

        } while (currentHeading < degrees && (runtime.seconds() < timeout));

        // Stop drive motors
        stopDriveMotors();

        // TODO: Should there be a pause when stopping motors?
//        stopDriveMotors(250); // Stop motors and delay 250 milliseconds
    }

    // Turn the robot a specific number of degrees (positive or negative) using Adafruit IMU
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    private void turnByDegreesIMU(double degrees, double speed) {
        // TODO: NOT TESTED
        String functionName = "turnByDegreesIMU";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        double gain = 0.7; // Tune this value as needed +/- for accurate turns
        double currentHeading;
        double timeout = 7.0;  // TODO: what is appropriate timeout value for a turn?
        double headingError;
        double driveSteering;
        double leftPower;
        double rightPower;
        Orientation angles;

        // Limit turnDegrees to 360 or less

        runtime.reset();
        do {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currentHeading = angles.firstAngle;
            LINEAR_OPMODE.telemetry.addData(functionName, "currentHeading = %.3f", currentHeading);
            LINEAR_OPMODE.telemetry.update();

            // Optimize turn direction
            if (currentHeading > 180.0) {
                currentHeading -= 360.0;
            }

            headingError = currentHeading - degrees;
            driveSteering = headingError * gain;

            leftPower = DEFAULT_TURN_SPEED + driveSteering;
            rightPower = DEFAULT_TURN_SPEED - driveSteering;

            leftPower = Range.clip(leftPower, 0.0, 1.0);
            rightPower = Range.clip(rightPower, 0.0, 1.0);

            drive(leftPower,rightPower);

        } while (currentHeading < degrees && (runtime.seconds() < timeout));

        // Stop drive motors
        stopDriveMotors();

        // TODO: Should there be a pause when stopping motors?
//        stopDriveMotors(250); // Stop motors and delay 250 milliseconds

    }

    // Turn the robot a specific number of degrees (positive or negative)
    // Note: Turns >= 360 degrees will be limited to < 360 degrees
    // Turns will always honor the direction, even if the turn could be optimized by turning in the
    // opposite direction.  E.g. a turn of -270 degrees will turn to the left instead of turning
    // to the right 90 degrees.
    void turnByDegrees(double degrees, double speed) {
        String functionName = "turnByDegrees";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

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
            try {
                throw new RobotConfigurationException("No gyro or IMU present!");
            } catch (RobotConfigurationException e) {
                e.printStackTrace();
            }
        }
    }

    // Turn robot a specific number of degrees (positive or negative) at the default speed
    void turnByDegrees(double degrees) {
        String functionName = "turnByDegrees";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        turnByDegrees(degrees, DEFAULT_TURN_SPEED);
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
    void fireAndArmChooChooLauncher() {
        // TODO: NOT TESTED
        String functionName = "fireAndArmChooChooLauncher";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        int newTarget;

        if (LINEAR_OPMODE == null) {
            try {
                throw(new RobotConfigurationException(functionName + ": Must be LinearOpMode!"));
            } catch (RobotConfigurationException e) {
                e.printStackTrace();
            }
            return;
        }

        // Get the current run mode and save it for restoring it later
        DcMotor.RunMode runMode = ChooChooMotor.getMode();

        // Set to RUN_TO_POSITION
        ChooChooMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Firing the launcher means one full rotation of the ChooChooMotor
        newTarget = ChooChooMotor.getCurrentPosition() + (int) (motorData.get(ChooChooMotor).encoderCountPerRevolution * CHOO_CHOO_MOTOR_GEAR_REDUCTION);
        ChooChooMotor.setTargetPosition(newTarget);

        ChooChooMotor.setPower(CHOO_CHOO_MOTOR_SPEED);

        while (ChooChooMotor.isBusy()) {
            LINEAR_OPMODE.telemetry.addData(functionName, "Firing launcher...");
            LINEAR_OPMODE.telemetry.update();
        }

        ChooChooMotor.setPower(0.0);

        // Reset motor run mode
        ChooChooMotor.setMode(runMode);
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
            DbgLog.msg("%s", functionName);
        }

        // Initialize the motor metadata map
        // TODO: is there a better place to put this initialization?
        motorMetadataMap.initialize();

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }

        // Define Motors
//        LFMotor   = hwMap.dcMotor.get("LFMotor");
//        RFMotor  = hwMap.dcMotor.get("RFMotor");
//        LMMotor = hwMap.dcMotor.get("LMMotor");
//        RMMotor = hwMap.dcMotor.get("RMMotor");
        LBMotor   = hwMap.dcMotor.get("LBMotor");
        RBMotor  = hwMap.dcMotor.get("RBMotor");
        ChooChooMotor = hwMap.dcMotor.get("ChooChooMotor");
        BeaterMotor = hwMap.dcMotor.get("BeaterMotor");
//        LiftMotor1 = hwMap.dcMotor.get("LiftMotor1");
//        LiftMotor2 = hwMap.dcMotor.get("LiftMotor2");
//        SlideMotor = hwMap.dcMotor.get("SlideMotor");

        // All Motor Map
        allMotorMap.put(LBMotor, "LBMotor");
        allMotorMap.put(RBMotor, "RBMotor");
        allMotorMap.put(ChooChooMotor, "ChooChooMotor");
        allMotorMap.put(BeaterMotor, "BeaterMotor");
//        allMotorMap.put(LiftMotor1, "LiftMotor1");
//        allMotorMap.put(LiftMotor2, "LiftMotor2");
//        allMotorMap.put(SlideMotor, "SlideMotor");

        // Drive motor map
        driveMotorMap.put(LBMotor, "LBMotor");
        driveMotorMap.put(RBMotor, "RBMotor");

        // Map the motors to their metadata, which includes encoder counts, etc.
//        motorData.put(LFMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(RFMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(LMMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(RMMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(LBMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(RBMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(ChooChooMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));
        motorData.put(BeaterMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.TETRIX));
//        motorData.put(LiftMotor1, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(LiftMotor2, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));
//        motorData.put(SlideMotor, motorMetadataMap.MOTOR_METADATA_MAP.get(MotorMetadata.MotorType.ANDYMARK_NEVEREST_40));

        // Set starting motor directions
//        LFMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        RFMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        LMMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        RMMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LBMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RBMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        ChooChooMotor.setDirection(DcMotor.Direction.REVERSE);
        BeaterMotor.setDirection(DcMotor.Direction.REVERSE);
//        LiftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        LiftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
//        SlideMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set mode for non-drive motors
        ChooChooMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BeaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LiftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set braking mode for drive motor
        // SDK default for year 2 is to put the motor in BRAKE mode.  Year 1 was FLOAT.
        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set braking mode for other non-drive motors
        ChooChooMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BeaterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LiftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set drive motors to zero power
        stopDriveMotors();

        // Set other motors to zero power
        ChooChooMotor.setPower(0.0);
        BeaterMotor.setPower(0.0);
//        LiftMotor1.setPower(0.0);
//        LiftMotor2.setPower(0.0);
//        SlideMotor.setPower(0.0);

        // *** No longer valid! See note for 1/3/2017 below.
        // Set motor max speed based on encoder counts from motor metadata
        // Count per revolution is dependent on the motor type
        // If not set, defaults to Tetrix  (1440) which impacts performance of AndyMark motors.
        // TODO: Should this be set to the max value or should it reserve 10% for PID?

        // *** Note: As of 1/3/2017, use of setMaxSpeed is no longer preferable because the
        // Modern Robotics implementation is opaque (hard to understand).
        // The newly preferred option is to attach each motor controller to a computer and
        // use the Modern Robotics Core Device Discovery utility to set each port to the
        // motor type of the motor that is to be attached to the robot.
        // Beware when swapping motor controllers to ensure that this step is taken or robot
        // behavior when using encoders may not be optimal.

//        LFMotor.setMaxSpeed((int)motorData.get(LFMotor).encoderCountPerRevolution);
//        RFMotor.setMaxSpeed((int)motorData.get(RFMotor).encoderCountPerRevolution);
//        LMMotor.setMaxSpeed((int)motorData.get(LFMotor).encoderCountPerRevolution);
//        RMMotor.setMaxSpeed((int)motorData.get(RFMotor).encoderCountPerRevolution);
//        LBMotor.setMaxSpeed((int)motorData.get(LBMotor).encoderCountPerRevolution);
//        RBMotor.setMaxSpeed((int)motorData.get(RBMotor).encoderCountPerRevolution);
//        ChooChooMotor.setMaxSpeed((int)motorData.get(ChooChooMotor).encoderCountPerRevolution);
//        BeaterMotor.setMaxSpeed((int)motorData.get(BeaterMotor).encoderCountPerRevolution);

        // Validate the number of motors is less than or equal to the maximum number allowed
        if (allMotorMap.size() > MAX_MOTORS_ALLOWED) {
            throw new RobotConfigurationException("Too many motors configured!");
        }
    }

    // Initialize servos
    private void initializeServos() throws RobotConfigurationException {
        String functionName = "initializeServos";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        // Add servo initialization here

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }
    }

    // Initialize sensors
    private void initializeSensors() throws RobotConfigurationException {
        String functionName = "initializeSensors";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        if (hwMap == null) {
            throw new RobotConfigurationException(functionName + ": HardwareMap is null");
        }

        // Get Sensors from Hardware Map
        cdim = hwMap.deviceInterfaceModule.get("cdim");
//        gyro = hwMap.gyroSensor.get("gyro");
        imu = hwMap.get(BNO055IMU.class, "imu");
//        color = hwMap.colorSensor.get("color");
        afcolor = hwMap.colorSensor.get("afcolor");
        ods = hwMap.opticalDistanceSensor.get("ods");
        allianceChannel = hwMap.digitalChannel.get("alliance");
        startPositionChannel = hwMap.digitalChannel.get("startposition");
        strategyChannel = hwMap.digitalChannel.get("strategy");
//        irBeamBreak = hwMap.digitalChannel.get("irBeamBreak");


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

        // Turn off CDIM red and blue LEDs
        setCdimLEDs(false, false);

        //
        // Alliance configuration switch
        //
        setAllianceFromSwitch();

        if (alliance == Alliance.BLUE) {
            setCdimLEDs(true, false);
        } else {
            setCdimLEDs(false, true);
        }

        //
        // Start Position configuration switch
        //
        if (startPositionChannel != null) {
            startPositionChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (startPositionChannel.getState()) {
                setStartPosition(StartPosition.POSITION1);
            } else {
                setStartPosition(StartPosition.POSITION2);
            }
        }
        else {
            setStartPosition(StartPosition.POSITION1);
        }

        //
        // Strategy configuration switch
        //
        if (strategyChannel != null) {
            strategyChannel.setMode(DigitalChannelController.Mode.INPUT);
            if (strategyChannel.getState()) {
                setStrategy(Strategy.STRATEGY1);
            } else {
                setStrategy(Strategy.STRATEGY2);
            }
        }
        else {
            setStrategy(Strategy.STRATEGY1);
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

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

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

    // Initialize Hardware interfaces
    // Use this version with non-LinearOpModes
    public void init(HardwareMap ahwMap) {
        String functionName = "init";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        setLoggingMode(LoggingMode.NONE);

        // Save reference to Hardware map
        hwMap = ahwMap;

        setGamepadDeadzone();

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

        // Add a LiftMechanism
//        lift = new LiftMechanism(LINEAR_OPMODE, LiftMotor1, LiftMotor2, motorData, 10.0);

        // Add a LinearSlide
//        slide = new LinearSlide(LINEAR_OPMODE, SlideMotor, motorData, 5.0, 0.5, 1.0, false);
    }

    // Initialize Hardware interfaces
    public void init(HardwareMap ahwMap, LinearOpMode om) {
        String functionName = "init";

        if (DEBUG_MODE) {
            DbgLog.msg("%s", functionName);
        }

        setLinearOpMode(om);
        init(ahwMap);

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

