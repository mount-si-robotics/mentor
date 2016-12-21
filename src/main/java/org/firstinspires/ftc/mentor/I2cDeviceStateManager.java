package org.firstinspires.ftc.mentor;

import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by markdolecki on 12/20/16.
 *
 * Based on FtcI2cDeviceState class posted on FTC forums by mikets of Titan Robotics Club
 * on 12/17/2016.
 *
 */

/* Example usage:
 *
 * public ModernRoboticsI2cRangeSensor rangeSensor;
 * public I2cDeviceStateManager rangeSensorState;
 *
 * // In init
 * rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
 * rangeSensorState = new I2cDeviceStateManager(ModernRoboticsI2cRangeSensor.class);
 * rangeSensorState.setEnabled(false); // Disable range sensor
 *
 * // When you need to re-enable the sensor
 * rangeSensorState.setEnabled(true);
 *
 * // When you are done with the sensor
 * rangeSensorState.setEnabled(false);
 *
 * NOTE: Re-enable the sensor in advance of when you need it so that the cached sensor data
 * has time to refresh.  This depends on refresh rate of the sensor (see sensor docs), but is
 * typically way less than 500 milliseconds.
 *
 * For older style devices such as the Modern Robotics Gyro, use the appropriate constructor.
 * WARNING: synch devices will block as they try to process remaining transactions before
 * disengaging.
 *
 */

public class I2cDeviceStateManager {
    private I2cDeviceSynch syncDevice = null;
    private int port = 0;
    private I2cController i2cController = null;
    private I2cController.I2cPortReadyCallback deviceCallback = null;
    private boolean deviceEnabled = true;

    public I2cDeviceStateManager(I2cControllerPortDevice device) {
        i2cController = device.getI2cController();
        port = device.getPort();
        deviceCallback = i2cController.getI2cPortReadyCallback(port);
        deviceEnabled = true;  // Assume device is enabled at the time this class is instantiated.
    }

    public I2cDeviceStateManager(I2cDeviceSynch device) {
        this.syncDevice = device;
    }

    public boolean isDeviceEnabled() {
        return deviceEnabled;
    }

    public void setEnabledState(boolean state) {
        if (deviceEnabled != state) {
            if (state) {
                if (syncDevice != null) {
                    syncDevice.engage();
                }
                else {
                    i2cController.registerForI2cPortReadyCallback(deviceCallback, port);
                }
            } else {
                if (syncDevice != null) {
                    syncDevice.disengage();
                }
                else {
                    i2cController.deregisterForPortReadyBeginEndCallback(port);
                }
            }
        }
    }
}
