package org.firstinspires.ftc.mentor;

/**
 * Created by markdolecki on 1/9/17.
 */ // TODO: Implement generic drive routine that calls the appropriate drive function
enum DriveTrain {
    TWO_WHEEL_REAR,
    TWO_WHEEL_CENTER,
    TWO_WHEEL_FRONT,
    FOUR_WHEEL,
    MECANUM_CARTESIAN,
    MECANUM_POLAR,
    NONE,
    NULL // Will simulate motor movement for debugging  TODO: Implement NULL mode for motors
}
