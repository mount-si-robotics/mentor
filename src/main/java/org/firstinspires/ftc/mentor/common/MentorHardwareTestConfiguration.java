package org.firstinspires.ftc.mentor.common;

import com.google.gson.Gson;

import java.util.Arrays;
import java.util.List;

/**
 * Created by markdolecki on 12/14/16.
 */

public class MentorHardwareTestConfiguration {
    // TODO: NOT TESTED

    // TODO: Add variables
    // E.g. Gamepad stick deadzone, motor types, etc.
    public List<String> headings = Arrays.asList("Functions", "Motor", "Sensors", "Servos");
    public List<String> functionTests = null;
    public List<String> motorTests = null;
    public List<String> sensorTests = null;
    public List<String> servoTests = null;

    public String serialize() {
        return new Gson().toJson(this);
    }
    public static MentorHardwareTestConfiguration deserialize(String data) {
        return new Gson().fromJson(data, MentorHardwareTestConfiguration.class);
    }

}
