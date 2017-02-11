package org.firstinspires.ftc.mentor.mechanisms;

import com.google.gson.internal.LazilyParsedNumber;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.mentor.common.MotorMetadata;

import java.util.Map;

public class BallLauncher {
    private boolean DEBUG = false;
    private LinearOpMode opMode;

    private DcMotor motor1;
    private DcMotor motor2;
    private Map<DcMotor, MotorMetadata> motorMetadata;

    private double LAUNCHER_GEAR_REDUCTION = 1.0;
    private double DEFAULT_LAUNCHER_SPEED = 0.5;
    private double LAUNCHER_WHEEL_DIAMETER = 2.5; // inches

    private int magazineSize = 1; // Number of balls launcher can hold

    private int numBallsLoaded = 1; // Number of balls loaded

    public enum LauncherState {
        EMPTY,
        LOADED,
        ARMED,
        LAUNCHING,
        LAUNCHED,
        ARMING,
        UNKNOWN
    }

    //
    // Constructors
    //

    private LauncherState launcherState = LauncherState.ARMED; // Default assumes in starting state

    // Default constructor
    private BallLauncher() {
        // Default constructor
    }

    public BallLauncher(LinearOpMode opMode, DcMotor motor1, DcMotor motor2, Map<DcMotor, MotorMetadata> metadata,
                        int magazineSize, boolean debug) {
        if (opMode == null) {
            opModeRequired();
        }
        this.opMode = opMode;

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motorMetadata = metadata;
        this.DEBUG = debug;
    }

    public LauncherState getLauncherState() {
        String functionName = "getLauncherState";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }
        return this.launcherState;
    }

    public LauncherState detectLauncherState() {
        String functionName = "detectLauncherState";

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
//            if (power > 0) {
//                return LiftState.RAISING;
//            }
//            else {
//                return LiftState.LOWERING;
//            }
        }

        // Assume retracted until sensor logic is added.
        return LauncherState.UNKNOWN;
    }

    public double getLauncherGearReduction() {
        String functionName = "getLauncherGearReduction";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return LAUNCHER_GEAR_REDUCTION;
    }

    public void setLauncherGearReduction(double gearReduction) {
        String functionName = "setLauncherGearReduction";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        LAUNCHER_GEAR_REDUCTION = gearReduction;
    }

    public double getLauncherSpeed() {
        String functionName = "getLauncherSpeed";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return DEFAULT_LAUNCHER_SPEED;
    }

    public void setLauncherSpeed(double launcherSpeed) {
        String functionName = "setLauncherSpeed";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        DEFAULT_LAUNCHER_SPEED = launcherSpeed;
    }

    public double getLauncherWheelDiameter() {
        String functionName = "getLauncherWheelDiameter";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return LAUNCHER_WHEEL_DIAMETER;
    }

    public void setLauncherWheelDiameter(double launcherWheelDiameter) {
        String functionName = "setLauncherWheelDiameter";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        LAUNCHER_WHEEL_DIAMETER = launcherWheelDiameter;
    }

    public int getMagazineSize() {
        String functionName = "getMagazineSize";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return magazineSize;
    }

    public void setMagazineSize(int magazineSize) {
        String functionName = "setMagazineSize";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        this.magazineSize = magazineSize;
    }

    public int getNumBallsLoaded() {
        String functionName = "getNumBallsLoaded";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        return numBallsLoaded;
    }

    public void setNumBallsLoaded(int numBalls) {
        String functionName = "setNumBallsLoaded";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        numBallsLoaded = numBalls;
    }

    public void loadBall() {
        String functionName = "loadBall";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        numBallsLoaded++;
    }

    public void armLauncher() {
        // TODO: Implement this
        String functionName = "armLauncher";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        launcherState = LauncherState.ARMING;



        launcherState = LauncherState.ARMED;

    }

    public void fireLauncher() {
        // TODO: Implement this
        String functionName = "fireLauncher";

        if (DEBUG) {
            DbgLog.msg("%s", functionName);
        }

        launcherState = LauncherState.LAUNCHING;

        launcherState = LauncherState.LAUNCHED;
        numBallsLoaded--;


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
