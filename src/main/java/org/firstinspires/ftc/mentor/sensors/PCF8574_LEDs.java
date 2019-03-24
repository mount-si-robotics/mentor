package org.firstinspires.ftc.mentor.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by markdolecki on 1/28/17.
 *
 * This class can be used to control up to eight individual LEDs or a seven segment numeric display.
 */

public class PCF8574_LEDs extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static String DEVICE_NAME = "PCF8574";

    private boolean DEBUG = false;

    protected static final int MUX_I2C_ADDRESS = 0x20; // Address for PCF8574 is 0x40

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(MUX_I2C_ADDRESS);

    private int currentState = 0;

    // Data pins on the PCF8574P
    public int P0 = (1 << 0);
    public int P1 = (1 << 1);
    public int P2 = (1 << 2);
    public int P3 = (1 << 3);
    public int P4 = (1 << 4);
    public int P5 = (1 << 5);
    public int P6 = (1 << 6);
    public int P7 = (1 << 7);

    // Bit masks for 7 segment LED display
    //    P0
    //  P5  P1
    //    P6
    //  P4  P2
    //    P3
    // Note: P7 is the dot
    private int Zero = P0 | P1 | P2 | P3 | P4 | P5;
    private int One = P1 | P2;
    private int Two = P0 | P1 | P3 | P4 | P6;
    private int Three = P0 | P1 | P2 | P3 | P6;
    private int Four = P1 | P2 | P5 | P6;
    private int Five = P0 | P2 | P3 | P5 | P6;
    private int Six = P0 | P2 | P3 | P4 | P5 | P6;
    private int Seven = P0 | P1 | P2;
    private int Eight = P0 | P1 | P2 | P3 | P4 | P5 | P6;
    private int Nine = P0 | P1 | P2 | P3 | P5 | P6;
    private int Dot = P7;


    // Assumes eight (8) LEDs are connected to the PCF8574P chip

    public PCF8574_LEDs(I2cDeviceSynch deviceClient, boolean isOwned) {
        super(deviceClient, isOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();

        super.registerArmingStateCallback(true);
    }


    /**
     * Actually carries out the initialization of the instance.
     *
     * @return Whether the initialization was successful or not
     */
    @Override
    protected boolean doInitialize() {
        // Nothing to do here
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
    // Functions
    //

    public void allOn() {
        writeData(0xff);
    }

    public void allOff() {
        writeData(0x00);
    }

    public int getCurrentState() {
        return currentState;
    }

    public void setLEDs(int mask) {
        int leds = Range.clip(mask, 0, 0xff);
        writeData(leds);
    }

    public void showDigit(int digit) {
        digit = Range.clip(digit,0,9);
        switch (digit) {
            case 0:
                writeData(Zero);
                break;
            case 1:
                writeData(One);
                break;
            case 2:
                writeData(Two);
                break;
            case 3:
                writeData(Three);
                break;
            case 4:
                writeData(Four);
                break;
            case 5:
                writeData(Five);
                break;
            case 6:
                writeData(Six);
                break;
            case 7:
                writeData(Seven);
                break;
            case 8:
                writeData(Eight);
                break;
            case 9:
                writeData(Nine);
                break;
            default:
                break;
        }
    }

    public void showDot(boolean show) {
        if (show) {
            writeData(Dot);
        }
        else {
            writeData(~Dot);
        }
    }

    //
    // Utility
    //

    private void writeData(int data) {
        int output = ~data;

        this.deviceClient.write8(0x00, output);
    }
}
