package org.firstinspires.ftc.mentor.mechanisms;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.mentor.common.MotorMetadata;

import java.util.Map;

/**
 * Created by markdolecki on 2/2/17.
 *
 * A Collector is a mechanism that contains one or two rotating beater bars that sweep
 * game components into the robot.
 *
 * Rotating forward collects items.
 *
 * Reversing the mechanism can free up a jammed item or can be used to push items away from the
 * robot.
 */

public class Collector {
    private boolean DEBUG = false;
    private LinearOpMode opMode;

    private DcMotor motor1;
    private DcMotor motor2;
    private Map<DcMotor, MotorMetadata> motorMetadata;

    private double COLLECTOR_GEAR_REDUCTION = 1.0;
    private double DEFAULT_COLLECTOR_SPEED = 0.5;
    private double COLLECTOR_WHEEL_DIAMETER = 2.5; // inches

    public enum CollectorState {
        STOPPED,
        FORWARD,
        REVERSE,
        UNKNOWN
    }

    //
    // Constructors
    //

    private CollectorState currentState = CollectorState.STOPPED; // Default is stopped

    // Default constructor
    private Collector() {
        // Default constructor
    }

    public Collector(LinearOpMode opMode, DcMotor motor1, DcMotor motor2, Map<DcMotor, MotorMetadata> metadata,
                     boolean debug) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorMetadata = metadata;
        this.DEBUG = debug;
    }

    public Collector(LinearOpMode opMode, DcMotor motor1, Map<DcMotor, MotorMetadata> metadata,
                     boolean debug) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        motor2 = null;
        this.motorMetadata = metadata;
        this.DEBUG = debug;
    }

    public CollectorState getCollectorState() {
        String functionName = "getCollectorState";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }
        return this.currentState;
    }

    public CollectorState detectCollectorState() {
        String functionName = "detectCollectorState";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        if (motor1 == null) {
            return CollectorState.UNKNOWN;
        }

        CollectorState state = CollectorState.STOPPED;

        if (motor1.isBusy()) {
            if (motor1.getPower() > 0) {
                // Assumes that if motor power > 0, the mechanism is moving forward
                currentState = CollectorState.FORWARD;
            }
            else {
                currentState = CollectorState.REVERSE;
            }
        }

        return currentState;
    }

    public double getCollectorGearReduction() {
        String functionName = "getCollectorGearReduction";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return COLLECTOR_GEAR_REDUCTION;
    }

    public void setCollectorGearReduction(double gearReduction) {
        String functionName = "setCollectorGearReduction";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        COLLECTOR_GEAR_REDUCTION = gearReduction;
    }

    public double getCollectorSpeed() {
        String functionName = "getCollectorSpeed";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return DEFAULT_COLLECTOR_SPEED;
    }

    public void setCollectorSpeed(double collectorSpeed) {
        String functionName = "setCollectorSpeed";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        DEFAULT_COLLECTOR_SPEED = collectorSpeed;
    }

    public double getCollectorWheelDiameter() {
        String functionName = "getCollectorWheelDiameter";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return COLLECTOR_WHEEL_DIAMETER;
    }

    public void setCollectorWheelDiameter(double collectorWheelDiameter) {
        String functionName = "setCollectorWheelDiameter";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        COLLECTOR_WHEEL_DIAMETER = collectorWheelDiameter;
    }


    private void runCollector(double speed) {
        String functionName = "runCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        if (motor1 != null) {
            motor1.setPower(speed);
        }
        if (motor2 != null) {
            motor2.setPower(speed);
        }
    }

    public void startCollector(double speed) {
        String functionName = "startCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        runCollector(speed);
        currentState = CollectorState.FORWARD;
    }

    public void startCollector() {
        String functionName = "startCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        runCollector(DEFAULT_COLLECTOR_SPEED);
        currentState = CollectorState.FORWARD;
    }

    public void reverseCollector(double speed) {
        String functionName = "reverseCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        // speed should be negative, so fix it if it is passed in as a positive value
        speed *= speed > 0 ? -1 : 1;

        runCollector(speed);
        currentState = CollectorState.REVERSE;
    }

    public void reverseCollector() {
        String functionName = "reverseCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        runCollector(DEFAULT_COLLECTOR_SPEED * -1);
        currentState = CollectorState.REVERSE;
    }

    public void stopCollector() {
        String functionName = "stopCollector";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        runCollector(0);
        currentState = CollectorState.STOPPED;
    }


    //
    // Utility Functions
    //

    private void opModeRequired() {
        String functionName = "opModeRequired";
        String msg = "A LinearOpMode is required";

        if (DEBUG) {
            RobotLog.d(msg);
        }
        try {
            throw new Exception(msg);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void unsupported(String msg) {
        String functionName = "unsupported";

        if (DEBUG) {
            RobotLog.d("%s not supported", functionName);
        }
        try {
            throw new Exception(msg);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
