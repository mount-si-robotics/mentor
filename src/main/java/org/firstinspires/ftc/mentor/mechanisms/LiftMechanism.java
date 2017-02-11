package org.firstinspires.ftc.mentor.mechanisms;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.mentor.common.MotorMetadata;

import java.util.Map;


/*
 * Class describing a lift mechanism that us used to raise a load to a specific height.
 *
 * Note: Assumes:
 * 1. Lift has two motors of the same motor type.
 * 2. Both motors have encoders.
 */
public class LiftMechanism {

    private boolean DEBUG = false;
    private LinearOpMode opMode;
    private double TARGET_HEIGHT_IN = 12.0; // Target lift height
    private DcMotor motor1;
    private DcMotor motor2;
    private Map<DcMotor, MotorMetadata> motorMetadata;
//    private MotorMetadata.MotorType motorType = MotorMetadata.MotorType.ANDYMARK_NEVEREST_40; // default to AndyMark Neverest 40
    private double LIFT_GEAR_REDUCTION = 1.0;
    private double DEFAULT_LIFT_SPEED = 0.5;
    private double SPOOL_DIAMETER = 2.5; // inches
    private double STRING_LENGTH = 14.0; // inches
    private double currHeight = 0;
    private boolean liftStuck = false;

    public enum LiftState {
        RETRACTED,
        RAISING,
        RAISED,
        LOWERING,
        UNKNOWN
    }

    private LiftState liftState = LiftState.RETRACTED; // Default assumes in starting state

    private LiftMechanism() {
        // Default constructor
    }

    public LiftMechanism(LinearOpMode opMode, DcMotor motor1, DcMotor motor2, Map<DcMotor, MotorMetadata> metadata, double height, boolean debug) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorMetadata = metadata;
        this.TARGET_HEIGHT_IN = height;
        this.DEBUG = debug;
    }

    public LiftMechanism(LinearOpMode opMode, DcMotor motor1, DcMotor motor2, Map<DcMotor, MotorMetadata> metadata, double height) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorMetadata = metadata;
        this.TARGET_HEIGHT_IN = height;
    }

    public LiftMechanism(LinearOpMode opMode, DcMotor motor1, DcMotor motor2, Map<DcMotor, MotorMetadata> metadata) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorMetadata = metadata;
    }

//    public LiftMechanism(LinearOpMode opMode, DcMotor motor1, DcMotor motor2) {
//        if (opMode == null) {
//            opModeRequired();
//        }
//        this.opMode = opMode;
//
//        this.motor1 = motor1;
//        this.motor2 = motor2;
//    }

    public LiftState getLiftState() {
        String functionName = "getLiftState";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }
        return this.liftState;
    }

    public LiftState detectLiftState() {
        String functionName = "detectLiftState";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        // TODO: Detect the state of the lift if possible
        // Ideally there is a way to use a sensor to detect which state the lift is in
        //
        unsupported(functionName);

        // Check a sensor to determine if lift is in its retracted state.
        // Options include a Hall sensor, IR Beam Break sensor, etc.
//        if (SOMESENSOR == SOMEVALUE) { // add the appropriate logic here.
//            return LiftState.RETRACTED;
//        }

        // If the motors are busy, then lift is either raising or lowering.
        if (motor1.isBusy() || motor2.isBusy()) {
            // Check motor power level for motor1
            // If motor one power level is < 0 / > 0, ????, then lift is extending
            // TODO: What is appropriate test here?


            double power = motor1.getPower();
            if (power > 0) {
                return LiftState.RAISING;
            }
            else {
                return LiftState.LOWERING;
            }
        }

        // If not retracted and motors are not moving, then lift is raised.
//        return LiftState.RAISED;


        // Assume retracted until sensor logic is added.
        return LiftState.RETRACTED;
    }

    // Set Lift motor power
    public void setLiftSpeed(double speed) {
        motor1.setPower(speed);
        motor2.setPower(speed);
    }

    public void stopLift() {
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    // Raise lift to a specific height
    public void raiseLift(double height) {
        String functionName = "raiseLift";
        int targetPosition; // In Encoder Ticks
        int targetPosition2; // In Encoder Ticks


        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (liftState != LiftState.RETRACTED || liftState == LiftState.UNKNOWN) {
            DbgLog.msg("LiftMechanism is in wrong state.");
            // Nothing to do
            return;
        }

        liftState = LiftState.RAISING;

        // Save the runmode state to restore later
        DcMotor.RunMode initialRunMode = this.motor1.getMode();
        DcMotor.RunMode initialRunMode2 = this.motor2.getMode();

        // Set mode to Run to Position
        this.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO: Confirm signs below
        targetPosition = (int)(this.motor1.getCurrentPosition() + height * motorMetadata.get(motor1).encoderCountPerRevolution * LIFT_GEAR_REDUCTION);
        targetPosition2 = (int)(this.motor2.getCurrentPosition() - height * motorMetadata.get(motor2).encoderCountPerRevolution * LIFT_GEAR_REDUCTION);

        motor1.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition2);

        setLiftSpeed(DEFAULT_LIFT_SPEED);

        // Add some logic to detect if lift is stuck moving and cannot finish
        while (opMode.opModeIsActive() && motor1.isBusy() && motor2.isBusy() && (! liftStuck)) {
            // Movement happens here...
            opMode.telemetry.addData("Lift", "Raising lift");
            opMode.telemetry.update();
        }

        // Stop motors
        stopLift();
        opMode.telemetry.addData("Lift", "Lift raised.");
        opMode.telemetry.update();

        // Restore mode
        this.motor1.setMode(initialRunMode);
        this.motor2.setMode(initialRunMode2);

        liftState = LiftState.RAISED;

        currHeight = height;
    }

    // Raise lift to the default configured height
    public void raiseLift() {
        String functionName = "raiseLift";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        raiseLift(TARGET_HEIGHT_IN);
    }

    // Retract the lift to starting position
    public void retractLift() {
        String functionName = "retractLift";
        int targetPosition; // In Encoder Ticks
        int targetPosition2; // In Encoder Ticks

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (liftState != LiftState.RAISED || liftState != LiftState.UNKNOWN) {
            DbgLog.msg("LiftMechanism is in wrong state.");
            // Nothing to do
            return;
        }

        liftState = LiftState.LOWERING;

        // Save the runmode state to restore later
        DcMotor.RunMode initialRunMode = this.motor1.getMode();
        DcMotor.RunMode initialRunMode2 = this.motor2.getMode();

        // Set mode to Run to Position
        this.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO: Confirm signs below
        targetPosition = (int)(this.motor1.getCurrentPosition() + (-1) * currHeight * motorMetadata.get(motor1).encoderCountPerRevolution * LIFT_GEAR_REDUCTION);
        targetPosition2 = (int)(this.motor2.getCurrentPosition() - (-1) * currHeight * motorMetadata.get(motor2).encoderCountPerRevolution * LIFT_GEAR_REDUCTION);

        motor1.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition2);

        setLiftSpeed(DEFAULT_LIFT_SPEED);

        // Add some logic to detect if lift is stuck moving and cannot finish
        while (opMode.opModeIsActive() && motor1.isBusy() && motor2.isBusy() && (! liftStuck)) {
            // Movement happens here...
            opMode.telemetry.addData("Lift", "Lowering lift");
            opMode.telemetry.update();
        }
        if (liftStuck) {
            liftState = LiftState.UNKNOWN;
        }

        // Stop motors
        stopLift();
        opMode.telemetry.addData("Lift", "Lift retracted.");
        opMode.telemetry.update();

        // Restore mode
        this.motor1.setMode(initialRunMode);
        this.motor2.setMode(initialRunMode2);

        liftState = LiftState.RETRACTED;
        currHeight = 0;
    }

    //
    // Utility Functions
    //

    private void opModeRequired() {
        String functionName = "opModeRequired";
        String msg = "A LinearOpMode is required";

        if (DEBUG) {
            DbgLog.msg(msg);
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
            DbgLog.msg("%s not supported", functionName);
        }
        try {
            throw new Exception(msg);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


}
