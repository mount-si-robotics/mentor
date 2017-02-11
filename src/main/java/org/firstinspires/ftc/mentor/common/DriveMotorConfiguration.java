package org.firstinspires.ftc.mentor.common;

/**
 * Created by markdolecki on 12/28/16.
 */

public class DriveMotorConfiguration {

    private double TicksPerRevolution = 0.0;  // Number of encoder ticks per motor revolution
    private double DriveGearReduction = 1.0;  // Gear ratio
    private double WheelDiameterInches = 0.0; // Diameter of the wheel
    private double TicksPerInch = 0.0;        // Number of encoder ticks to drive one inch
    private double DegreesPerTick = 0.0;      // Degrees motor turns per encoder tick
    private double DriveSpeed = 0.5;          // Default drive speed
    private double TurnSpeed = 0.5;           // Default turn speed

    public DriveMotorConfiguration(double ticksPerRevolution, double driveGearReduction, double wheelDiameterInches, double ticksPerInch, double degreesPerTick, double driveSpeed, double turnSpeed) {
        TicksPerRevolution = ticksPerRevolution;
        DriveGearReduction = driveGearReduction;
        WheelDiameterInches = wheelDiameterInches;
        TicksPerInch = ticksPerInch;
        DegreesPerTick = degreesPerTick;
        DriveSpeed = driveSpeed;
        TurnSpeed = turnSpeed;
    }

    public DriveMotorConfiguration(double ticksPerRevolution, double driveGearReduction, double wheelDiameterInches, double ticksPerInch, double degreesPerTick) {
        TicksPerRevolution = ticksPerRevolution;
        DriveGearReduction = driveGearReduction;
        WheelDiameterInches = wheelDiameterInches;
        TicksPerInch = ticksPerInch;
        DegreesPerTick = degreesPerTick;
    }

    public double getTicksPerRevolution() {
        return TicksPerRevolution;
    }

    public void setTicksPerRevolution(double ticksPerRevolution) {
        TicksPerRevolution = ticksPerRevolution;
    }

    public double getDriveGearReduction() {
        return DriveGearReduction;
    }

    public void setDriveGearReduction(double driveGearReduction) {
        DriveGearReduction = driveGearReduction;
    }

    public double getWheelDiameterInches() {
        return WheelDiameterInches;
    }

    public void setWheelDiameterInches(double wheelDiameterInches) {
        WheelDiameterInches = wheelDiameterInches;
    }

    public double getTicksPerInch() {
        return TicksPerInch;
    }

    public void setTicksPerInch(double ticksPerInch) {
        TicksPerInch = ticksPerInch;
    }

    public double getDegreesPerTick() {
        return DegreesPerTick;
    }

    public void setDegreesPerTick(double degreesPerTick) {
        DegreesPerTick = degreesPerTick;
    }

    public double getDriveSpeed() {
        return DriveSpeed;
    }

    public void setDriveSpeed(double driveSpeed) {
        DriveSpeed = driveSpeed;
    }

    public double getTurnSpeed() {
        return TurnSpeed;
    }

    public void setTurnSpeed(double turnSpeed) {
        TurnSpeed = turnSpeed;
    }


}
