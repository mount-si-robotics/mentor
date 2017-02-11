package org.firstinspires.ftc.mentor.sensors;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

public class Multiplexer extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static String DEVICE_NAME = "Adafruit Multiplexer TCA9548A";

    private boolean DEBUG = false;

    private static final int MUX_I2C_ADDRESS = 0x70; // Address for mux is 0xE0

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(MUX_I2C_ADDRESS);

    protected Multiplexer(I2cDeviceSynch deviceClient, boolean isOwned) {
        super(deviceClient, isOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();

        super.registerArmingStateCallback();
    }


    /**
     * Actually carries out the initialization of the instance.
     *
     * @return Whether the initialization was successful or not
     */
    @Override
    protected boolean doInitialize() {


        return true;
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return DEVICE_NAME;
    }

    //
    // Mux select device 0-7
    //
    public void muxSelect(int device) {
        if (device < 0 || device > 7) {
            this.deviceClient.write8(0, (1 << device));

        }
    }


}
