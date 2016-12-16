package org.firstinspires.ftc.mentor;

import com.google.gson.Gson;

/**
 * Created by markdolecki on 12/14/16.
 */

public class MentorHardwareRobotConfiguration {

    // Robot configuration data class allows the use of a JSON data file stored on the robot
    // controller phone to control various hardware parameters.  By swapping the configuration
    // data file, different robot behaviours can be tested without requiring modification to the
    // robot source code files.
    // Note: This configuration file should not be necessary, meaning that all robot configuration
    // parameters should be initialized with reasonable defaults (fail safe), and all parameters
    // should be modifiable either during robot initialization OR dynamically by an OpMode.

    // TODO: NOT TESTED

    // TODO: Add variables
    // E.g. Gamepad stick deadzone, motor types, etc.

    public String serialize() {
        return new Gson().toJson(this);
    }
    public static MentorHardwareRobotConfiguration deserialize(String data) {
        return new Gson().fromJson(data, MentorHardwareRobotConfiguration.class);
    }
}
