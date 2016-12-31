package org.firstinspires.ftc.mentor;

/**
 * Created by markdolecki on 12/19/16.
 */



public class MotorMetadata {
    String name = null;
    MotorType type = null;
    double voltage = 0;
    double encoderCountPerRevolution = 0;
    String gearboxReduction = null;
    double maxRPM = 0;
    String gearboxOutputPower = null;
    String stallTorque = null;
    String stallCurrent = null;
    String gearboxBreakForce = null;
    String minBackdriveTorque = null;

    // List of FTC Legal Motor Types
    enum MotorType {
        ANDYMARK_NEVEREST_20,
        ANDYMARK_NEVEREST_40,
        ANDYMARK_NEVEREST_60,
        ANDYMARK_NEVEREST_3PT7,
        TETRIX,
        OTHER,
        NONE
    }

    public MotorMetadata() {
        // Default constructor
    }

    public MotorMetadata(String name, MotorType type, double voltage, double encoderCountPerRevolution, String gearboxReduction, double maxRPM, String gearboxOutputPower, String stallTorque, String stallCurrent, String gearboxBreakForce, String minBackdriveTorque) {
        this.name = name;
        this.type = type;
        this.voltage = voltage;
        this.encoderCountPerRevolution = encoderCountPerRevolution;
        this.gearboxReduction = gearboxReduction;
        this.maxRPM = maxRPM;
        this.gearboxOutputPower = gearboxOutputPower;
        this.stallTorque = stallTorque;
        this.stallCurrent = stallCurrent;
        this.gearboxBreakForce = gearboxBreakForce;
        this.minBackdriveTorque = minBackdriveTorque;
    }
}
