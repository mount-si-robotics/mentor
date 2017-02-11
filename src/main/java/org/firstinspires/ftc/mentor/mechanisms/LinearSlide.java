package org.firstinspires.ftc.mentor.mechanisms;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.mentor.common.MotorMetadata;

import java.util.Map;

public class LinearSlide {

    private LinearOpMode opMode = null;
    private DcMotor slideMotor = null; // One motor with encoder
    private Map<DcMotor, MotorMetadata> motorMetadata;
    private boolean DEBUG = false;
    private double slideCircumference = 1.0;  // Inches
    private double slideGearReduction = 1.0; // Ratio
    private double extendDistance = 1.0; // Inches
    private double speed = 0.5; // Default slide speed
    private boolean slideStuck = false;
    private double currentSlidePosition = 0; // Inches


    public enum SlideState {
        RETRACTED,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        UNKNOWN
    }

    private SlideState slideState = SlideState.RETRACTED;  // Assume slide is in retracted state

    /*
     * Constructors
     */

    public LinearSlide(LinearOpMode opMode, DcMotor slideMotor, Map<DcMotor, MotorMetadata> metadata, double extendDistance, double speed, double slideCircumference, boolean isExtended) {
        this.opMode = opMode;
        this.slideMotor = slideMotor;
        this.motorMetadata = metadata;
        this.extendDistance = extendDistance;
        this.speed = speed;
        this.slideCircumference = slideCircumference;
        if (isExtended) {
            this.slideState = SlideState.EXTENDED;
        }
        else {
            this.slideState = SlideState.RETRACTED;
        }
    }

    // Use this constructor when slide state starts in the extended position
    public LinearSlide(LinearOpMode opMode, DcMotor slideMotor, Map<DcMotor, MotorMetadata> metadata, boolean isExtended) {
        this.opMode = opMode;
        this.slideMotor = slideMotor;
        this.motorMetadata = metadata;
        if (isExtended) {
            this.slideState = SlideState.EXTENDED;
        }
        else {
            this.slideState = SlideState.RETRACTED;
        }
    }

    public LinearSlide(LinearOpMode opMode, DcMotor slideMotor, Map<DcMotor, MotorMetadata> metadata) {
        this.opMode = opMode;
        this.slideMotor = slideMotor;
        this.motorMetadata = metadata;
    }

    /*
     * Functions
     */

    public SlideState getSlideState() {
        String functionName = "getSlideState";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }
        return slideState;
    }

    public void setSlideState(SlideState state) {
        String functionName = "setSlideState";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.slideState = state;
    }

    public double getSpeed() {
        String functionName = "getSpeed";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return speed;
    }

    public void setSpeed(double speed) {
        String functionName = "setSpeed";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.speed = speed;
    }

    public double getSlideCircumference() {
        String functionName = "getSlideCircumference";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return slideCircumference;
    }

    public void setSlideCircumference(double circumference) {
        String functionName = "setSlideCircumference";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.slideCircumference = circumference;
    }

    public double getExtendDistance() {
        String functionName = "getExtendDistance";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return extendDistance;
    }

    public void setExtendDistance(double extendDistance) {
        String functionName = "setExtendDistance";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.extendDistance = extendDistance;
    }

    // Try to figure out what state the slide is in.
    // Return the state AND set the internal state to the value discovered.
    public SlideState determineSlideState() {
        String functionName = "determineSlideState";
        SlideState state = SlideState.UNKNOWN;

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        unsupported("determineState not implemented yet.");

//        if (SENSOR DETECTS SLIDE IN RETRACTED POSITION) {
//            state = SlideState.RETRACTED;
//        }
//        else {
//            state = SlideState.EXTENDED;
//        }

        // if slide motor is moving, then transition state depends on which direction it is moving
        if (slideMotor.isBusy()) {
            if (slideMotor.getPower() > 0) {
                // Extending
                state = SlideState.EXTENDING;
            }
            else {
                state = SlideState.RETRACTING;
            }

        }

        // Set slideState to the discovered state
        slideState = state;

        return slideState;
    }

    public double getSlideGearReduction() {
        String functionName = "getSlideGearReduction";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return slideGearReduction;
    }

    public void setSlideGearReduction(double slideGearReduction) {
        String functionName = "setSlideGearReduction";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.slideGearReduction = slideGearReduction;
    }

    public double getCurrentSlidePosition() {
        return currentSlidePosition;
    }

    public void stopSlide() {
        slideMotor.setPower(0);
    }


    public void extendSlide() {
        String functionName = "extendSlide";
        int targetPosition;

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (slideState != SlideState.RETRACTED || slideState == SlideState.UNKNOWN) {
            DbgLog.msg("LinearSlide is in wrong state");

            return;
        }

        slideState = SlideState.EXTENDING;

        // Move Slide
        // Save the runmode state to restore later
        DcMotor.RunMode initialRunMode = this.slideMotor.getMode();

        // Set mode to Run to Position
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO: Confirm sign below
        targetPosition = (int)(this.slideMotor.getCurrentPosition() + extendDistance * motorMetadata.get(slideMotor).encoderCountPerRevolution * slideGearReduction);

        slideMotor.setTargetPosition(targetPosition);

        slideMotor.setPower(speed);

        // Add some logic to detect if lift is stuck moving and cannot finish
        while (opMode.opModeIsActive() && slideMotor.isBusy() && (! slideStuck)) {
            // Movement happens here...
            opMode.telemetry.addData("LinearSlide", "Extending slide...");
            opMode.telemetry.update();
        }

        // Stop motors
        stopSlide();
        opMode.telemetry.addData("LinearSlide", "Slide extended.");
        opMode.telemetry.update();

        // Restore mode
        this.slideMotor.setMode(initialRunMode);

        slideState = SlideState.EXTENDED;
        currentSlidePosition = extendDistance;
    }

    public void retractSlide() {
        String functionName = "retractSlide";
        int targetPosition;

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        if (slideState != SlideState.EXTENDED || slideState == SlideState.UNKNOWN) {
            DbgLog.msg("LinearSlide is in wrong state");
            return;
        }

        slideState = SlideState.RETRACTING;

        // Save the runmode state to restore later
        DcMotor.RunMode initialRunMode = this.slideMotor.getMode();

        // Set mode to Run to Position
        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO: Confirm sign below
        targetPosition = (int)(this.slideMotor.getCurrentPosition() - currentSlidePosition * motorMetadata.get(slideMotor).encoderCountPerRevolution * slideGearReduction);

        slideMotor.setTargetPosition(targetPosition);

        slideMotor.setPower(speed);

        // Add some logic to detect if lift is stuck moving and cannot finish
        while (opMode.opModeIsActive() && slideMotor.isBusy() && (! slideStuck)) {
            // Movement happens here...
            opMode.telemetry.addData("LinearSlide", "Retracting slide");
            opMode.telemetry.update();
        }

        // Stop motors
        stopSlide();
        opMode.telemetry.addData("LinearSlide", "Slide retracted.");
        opMode.telemetry.update();

        // Restore mode
        this.slideMotor.setMode(initialRunMode);

        slideState = SlideState.RETRACTED;
    }

    /*
     * Utilities
     */

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
