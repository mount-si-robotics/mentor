package org.firstinspires.ftc.mentor.common;

/**
 * Created by markdolecki on 1/9/17.
 */ // TODO: Implement generic drive routine that calls the appropriate drive function
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
