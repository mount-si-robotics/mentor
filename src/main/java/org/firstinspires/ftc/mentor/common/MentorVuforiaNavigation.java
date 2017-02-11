package org.firstinspires.ftc.mentor.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.mentor.HardwareMentor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.mentor.R;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;


/**
 * Created by markdolecki on 1/4/17.
 *
 * Based on Robot_Navigation.java example from philbot
 * Adapted for Snoqualmie Valley Robotics by Mark Dolecki
 *
 */

public class MentorVuforiaNavigation {
    private static final int NUM_TARGETS = 4; // Number of Vuforia targets

    private static final boolean DISPLAY_CAMERA = false; // If true, camera view will display on robot
                                                         // controller phone. Performance will be slower
                                                         // when displaying the camera.

    // Update this value with the actual robot width
    private static final double ROBOT_WIDTH_INCHES = 17.0;
    private static final double ROBOT_WIDTH_MM = ROBOT_WIDTH_INCHES * DistanceUnit.mmPerInch;

    // FTC Field width in millimeters
    // The FTC field is ~11'10" center-to-center of the glass panels
    private static final float mmFTCFieldWidth  = (12*12 - 2) * (float)DistanceUnit.mmPerInch;

    private static final double ON_AXIS = 10; // Within 1.0 cm on axis
    private static final double  OFFSET_ERROR   =  20;      // Within 2.0 cm of final target

    // Uncomment one of the following lines to determine whether to use the front or back camera
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;

    private LinearOpMode opMode = null;
    private HardwareMentor hardwareMentor = null;
    private VuforiaTrackables targets;
    private boolean targetFound;
    private String targetName;

    // Do NOT use the key below!  Get your own!
    // Key below is registered to Mark Dolecki.
    private String VUFORIA_KEY = "AaUHMEr/////AAAAGTV5o70tjUyckRNm8KD0JNUpURBXvV4FXi5sxMH/WpoWEPKm2xWXPG1sR0f6D4uRvFj1YEk+U4+Eqac7+IchrkEf8CfsAbG5gE0dsHdGI2jMBerZ/SlVZADiiMFMSlitBzf7By2v0e/ZaioU8DGs3vkzyqsS1BeCEv7eEjBEPpzOg/uGRZLB0lMBuDC8dv3ZPOS1Ts1Y+7hTIGdA2Q/XBh+4RG/WFzwag2qbjYGcYiAsyUfThlHdGbKnG70tT9f72ava357/3VVW3w6NyDTI509bCVrVUOhNZKmQLLc2xYuYAn6PSyokNSc8vLEN1f51Y82z4/k3qYzD1w71LymvqarSKMtwWh7TVR+MB7lEsBS/";

    // Position of the camera in relation to the robot
    // Update these values for your robot!!!
    private static final int CAMERA_FORWARD_DISPLACEMENT  = (int) (8.5 * DistanceUnit.mmPerInch); //110;   // Camera is 110 mm in front of robot center
    private static final int CAMERA_VERTICAL_DISPLACEMENT = (int) (14.5 * DistanceUnit.mmPerInch); //200;   // Camera is 200 mm above ground
    private static final int CAMERA_LEFT_DISPLACEMENT     = (int) (-1.5 * DistanceUnit.mmPerInch);     // Camera is ON the robots center line

    private static final double  YAW_GAIN       =  0.018;   // Rate at which we respond to heading error
    private static final double  LATERAL_GAIN   =  0.0027;  // Rate at which we respond to off-axis error
    private static final double  AXIAL_GAIN     =  0.0017;  // Rate at which we respond to target distance errors

    // Robot position and bearing values
    private double robotX;          // X distance from robot center to target
    private double robotY;          // Y distance from robot center to target
    private double robotBearing;    // robot rotation around Z axis. Counter-clockwise is positive
    private double targetRange;     // distance from robot center to target in millimeters
    private double targetBearing;   // heading of the target from the center point of the robot
    private double relativeBearing; // heading to the target from the robots current bearing.
                                    // If relativeBearing is positive, robot must rotate counter-
                                    // clockwise to point at the target

    // Field Element Target Locations
    OpenGLMatrix blueNearTargetLocation;
    OpenGLMatrix blueFarTargetLocation;
    OpenGLMatrix redNearTargetLocation;
    OpenGLMatrix redFarTargetLocation;
    OpenGLMatrix centerGoalTargetLocation;


    public MentorVuforiaNavigation() {
        // Default constructor
        targetFound = false;
        targetName = null;
        targets = null;

        robotX = 0;
        robotY = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
    }

    // Get robot X coordinate
    public double getRobotX() {
        String functionName = "getRobotX";

        return robotX;
    }

    // Get robot Y coordinate
    public double getRobotY() {
        String functionName = "getRobotY";

        return robotY;
    }

    // Get robot bearing
    public double getRobotBearing() {
        String functionName = "getRobotBearing";

        return robotBearing;
    }

    // Get Target Range
    public double getTargetRange() {
        String functionName = "getTargetRange";

        return targetRange;
    }

    // Get Target Bearing
    public double getTargetBearing() {
        String functionName = "getTargetBearing";

        return targetBearing;
    }

    // Get Relative Bearing
    public double getRelativeBearing() {
        String functionName = "getRelativeBearing";

        return relativeBearing;
    }

    // Set target locations for various field elements
    public void setFieldElementLocations() {
        // TODO; NOT TESTED
        String functionName = "setFieldElementLocations";

        // TODO: Update values to actual positions for all elements below!!!
        // TODO:
        // TODO:
        blueNearTargetLocation = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        blueFarTargetLocation = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redNearTargetLocation = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redFarTargetLocation = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        centerGoalTargetLocation = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 0));
    }

    // Initialize Vuforia
    public void initialize(LinearOpMode om) {
        String functionName = "initialize";
        VuforiaLocalizer.Parameters parameters;

        opMode = om;
        //hardwareMentor = hm;

        // Choose one of the following two options
        if (DISPLAY_CAMERA) {
            // Display camera view on the robot controller phone screen.
            // Performance will be slower.
            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        }
        else {
            // Camera view will not be displayed.
            // Performance will be faster.
            parameters = new VuforiaLocalizer.Parameters();
        }

        // Get your own Vuforia key at  https://developer.vuforia.com/license-manager
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Load the data sets that for the trackable objects we wish to track.
        // These particular data sets are stored in the 'assets' part of our application
        // They represent the four image targets used in the 2016-17 FTC game.
        targets = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        targets.get(0).setName("Blue Near");  // wheels
        targets.get(1).setName("Red Far");    // tools
        targets.get(2).setName("Blue Far");   // legos
        targets.get(3).setName("Red Near");   // gears

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        List<VuforiaTrackable> allTrackables = new ArrayList<>();
        allTrackables.addAll(targets);

        // create an image translation/rotation matrix to be used for all images
        // Essentially put all the image centers 6" above the 0:0:0 origin,
        // but rotate them so they along the -X axis.
        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 0, 150)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));


        // Create a transformation matrix describing where the phone is on the robot.
        //
        // The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out
        // along the Y axis.
        //
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of
        // the phone pointing to the LEFT side of the Robot.  If we consider that the camera and
        // screen will be in "Landscape Mode" the upper portion of the screen is closest to the
        // front of the robot.
        //
        // If using the rear (High Res) camera:
        // We need to rotate the camera around it's long axis to bring the rear camera forward.
        // This requires a negative 90 degree rotation on the Y axis
        //
        // If using the Front (Low Res) camera
        // We need to rotate the camera around it's long axis to bring the FRONT camera forward.
        // This requires a Positive 90 degree rotation on the Y axis
        //
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but 110 mm forward of the middle of the
        // robot, and 200 mm above ground level.

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90, 0, 0));

        // Set the all the targets to have the same location and camera orientation
        for (VuforiaTrackable trackable : allTrackables)
        {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

    }

    public void addNavTelemetry() {
        if (targetFound)
        {
            // Display the current visible target name, robot info, target info, and required robot action.
            opMode.telemetry.addData("Visible", targetName);
            opMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)",
                    robotX, robotY, robotBearing);
            opMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);
            opMode.telemetry.addData("- Turn    ", "%s %4.0f°",  relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            opMode.telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            opMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
        }
        else
        {
            opMode.telemetry.addData("Visible", "- - - -" );
        }
    }

    // Start target tracking
    public void startTargetTracking() {
        String functionName = "startTargetTracking";

        if (targets != null) {
            targets.activate();
        }
    }

    // Stop target tracking
    public void stopTargetTracking() {
        String functionName = "stopTargetTracking";

        if (targets != null) {
            targets.deactivate();
        }
    }

    // Move to target
    public boolean moveToTarget(double standOffDistance) {
        String functionName = "moveToTarget";

        // TODO: Implement this
        boolean closeEnough;

        // Priority #1 Rotate to always be pointing at the target (for best target retention).
        double Y  = (relativeBearing * YAW_GAIN);

        // Priority #2  Drive laterally based on distance from X axis (same as y value)
        double L  =(robotY * LATERAL_GAIN);

        // Priority #3 Drive forward based on the desiredHeading target standoff distance
        double A  = (-(robotX + standOffDistance) * AXIAL_GAIN);

        // TODO: Move the robot
        // Send the desired axis motions to the robot hardware.
//        robot.setYaw(Y);
//        robot.setAxial(A);
//        robot.setLateral(L);

        // Determine if we are close enough to the target for action.
        closeEnough = ( (Math.abs(robotX + standOffDistance) < OFFSET_ERROR) &&
                (Math.abs(robotY) < ON_AXIS));

        return (closeEnough);
    }

    // Are any targets visible?
    public boolean anyTargetsVisible() {
        String functionName = "anyTargetsVisible";

        int targetNum = 0;

        while (targetNum < NUM_TARGETS) {
            if (isTargetVisible(targetNum)) {
                return true;
            }
            targetNum++;
        }

        return false;
    }

    boolean isTargetVisible(int targetNum) {
        String functionName = "isTargetVisible";

        VuforiaTrackable target = targets.get(targetNum);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location;

        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();


            // Check for robot location
            location = listener.getUpdatedRobotLocation();

            if (location != null) {
                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }
        }

        return targetFound;
    }



}
