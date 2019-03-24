package org.firstinspires.ftc.mentor.sensors;

import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.mentor.common.RobotConfigurationException;

import java.nio.ByteOrder;

import static java.lang.Math.atan2;

/**
 * Created by markdolecki on 1/20/17.
 */

//@I2cSensor(name = "GY271", xmlTag = "HMC5883LGY271", description = "A GY271 Compass")
public class HMC5883LGY271 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements CompassSensor {

    private final static String DEVICE_NAME = "HMC5883LGY271";
    private final static boolean DEBUG = false;

    public final static byte HMC5883L_ADDRESS = 0x3C; //0x1E; // this device only has one address
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(HMC5883L_ADDRESS);

    public enum Register {
        FIRST(0x00),
        HMC5883L_RA_CONFIG_A(0x00),
        HMC5883L_RA_CONFIG_B(0x01),
        HMC5883L_RA_MODE(0x02),
        HMC5883L_RA_DATAX_H(0x03),
        HMC5883L_RA_DATAX_L(0x04),
        HMC5883L_RA_DATAZ_H(0x05),
        HMC5883L_RA_DATAZ_L(0x06),
        HMC5883L_RA_DATAY_H(0x07),
        HMC5883L_RA_DATAY_L(0x08),
        HMC5883L_RA_STATUS(0x09),
        HMC5883L_RA_ID_A(0x0A),
        HMC5883L_RA_ID_B(0x0B),
        HMC5883L_RA_ID_C(0x0C),
        LAST(HMC5883L_RA_ID_C.bVal),
        COMMAND(0x09), // TODO: VERIFY THIS ADDRESS!!!
        UNKNOWN(-1);

        public byte bVal;

        Register(int bVal) {
            this.bVal = (byte) bVal;
        }
    }

    public final static int HMC5883L_CRA_AVERAGE_BIT = 6;
    public final static int HMC5883L_CRA_AVERAGE_LENGTH = 2;
    public final static int HMC5883L_CRA_RATE_BIT = 4;
    public final static int HMC5883L_CRA_RATE_LENGTH = 3;
    public final static int HMC5883L_CRA_BIAS_BIT = 1;
    public final static int HMC5883L_CRA_BIAS_LENGTH = 2;

    public final static byte HMC5883L_AVERAGING_1 = 0x00;
    public final static byte HMC5883L_AVERAGING_2 = 0x01;
    public final static byte HMC5883L_AVERAGING_4 = 0x02;
    public final static byte HMC5883L_AVERAGING_8 = 0x03;

    public final static byte HMC5883L_RATE_0P75 = 0x00;
    public final static byte HMC5883L_RATE_1P5 = 0x01;
    public final static byte HMC5883L_RATE_3 = 0x02;
    public final static byte HMC5883L_RATE_7P5 = 0x03;
    public final static byte HMC5883L_RATE_15 = 0x04;
    public final static byte HMC5883L_RATE_30 = 0x05;
    public final static byte HMC5883L_RATE_75 = 0x06;

    public final static byte HMC5883L_BIAS_NORMAL = 0x00;
    public final static byte HMC5883L_BIAS_POSITIVE = 0x01;
    public final static byte HMC5883L_BIAS_NEGATIVE = 0x02;

    public final static int HMC5883L_CRB_GAIN_BIT = 7;
    public final static int HMC5883L_CRB_GAIN_LENGTH = 3;

    public enum Gain {
        HMC5883L_GAIN_1370(0x00),
        HMC5883L_GAIN_1090(0x01),
        HMC5883L_GAIN_820(0x02),
        HMC5883L_GAIN_660(0x03),
        HMC5883L_GAIN_440(0x04),
        HMC5883L_GAIN_390(0x05),
        HMC5883L_GAIN_330(0x06),
        HMC5883L_GAIN_220(0x07);

        public byte bVal;

        Gain(int bVal) {
            this.bVal = (byte) bVal;
        }
    }
    public final static byte HMC5883L_GAIN_1370 = 0x00;
    public final static byte HMC5883L_GAIN_1090 = 0x01;
    public final static byte HMC5883L_GAIN_820 = 0x02;
    public final static byte HMC5883L_GAIN_660 = 0x03;
    public final static byte HMC5883L_GAIN_440 = 0x04;
    public final static byte HMC5883L_GAIN_390 = 0x05;
    public final static byte HMC5883L_GAIN_330 = 0x06;
    public final static byte HMC5883L_GAIN_220 = 0x07;

    public final static int HMC5883L_MODEREG_BIT = 1;
    public final static int HMC5883L_MODEREG_LENGTH = 2;

    public final static byte HMC5883L_MODE_CONTINUOUS = 0x00;
    public final static byte HMC5883L_MODE_SINGLE = 0x01;
    public final static byte HMC5883L_MODE_IDLE = 0x02;

    public enum Mode {
        CONTINUOUS(0x00),
        SINGLE(0x01),
        IDLE(0x02),
        IDLE2(0x03),
        UNKNOWN(-1);
//        public final static byte HMC5883L_MODE_CONTINUOUS = 0x00;
//        public final static byte HMC5883L_MODE_SINGLE = 0x01;
//        public final static byte HMC5883L_MODE_IDLE = 0x02;
        public byte bVal;

        Mode(int bVal) {
            this.bVal = (byte) bVal;
        }

        public static Mode fromByte(byte b) {
            for (Mode mode : values()) {
                if (mode.bVal == b) return mode;
            }
            return UNKNOWN;
        }
    }

    public final static int HMC5883L_STATUS_LOCK_BIT = 1;
    public final static int HMC5883L_STATUS_READY_BIT = 0;

    public enum Command
    {
        NORMAL(0x00),
        CALIBRATE_IRON(0x43),
        ACCEL_NULL_X(0x58),
        ACCEL_NULL_Y(0x59),
        ACCEL_NULL_Z(0x5A),
        ACCEL_GAIN_ADJUST(0x47),
        MEASURE_TILT_UP(0x55),
        MEASURE_TILT_DOWN(0x44),
        WRITE_EEPROM(0x57),
        CALIBRATION_FAILED(0x46),
        UNKNOWN(-1);

        public byte bVal;
        Command(int bVal) { this.bVal = (byte)bVal; }
        public static Command fromByte(byte b) {
            for (Command command : values()) {
                if (command.bVal == b) return command;
            }
            return UNKNOWN;
        }
    }

    /*
     * Constructor
     */

    public HMC5883LGY271(I2cDeviceSynch deviceClient, boolean isOwned) {
        super(deviceClient, isOwned);

        this.setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        RobotLog.d("I2c address: %d", this.deviceClient.getI2cAddress().get8Bit());

        this.deviceClient.engage();

        super.registerArmingStateCallback(true);

    }

    public void setOptimalReadWindow() {
        String functionName = "setOptimalReadWindow";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

//        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
//                Register.FIRST.bVal,
//                Register.LAST.bVal - Register.FIRST.bVal + 1,
//                I2cDeviceSynch.ReadMode.REPEAT);
//        this.deviceClient.setReadWindow(readWindow);
    }

    /**
     * Actually carries out the initialization of the instance.
     *
     * @return Whether the initialization was successful or not
     */
    @Override
    public boolean doInitialize() {
        String functionName = "doInitialize";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        // Enable the compass sensor
        this.deviceClient.write8(Register.HMC5883L_RA_CONFIG_A.bVal, 0x00);

        // Set the gain to a known level
        this.setGain(HMC5883L_GAIN_1370);

        return true;
    }

    /**
     * Get the current direction, in degrees
     *
     * @return current direction, in degrees
     */
    @Override
    public double getDirection() {
        String functionName = "getDirection";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }
        return atan2(getHeadingY(), getHeadingX());

    }

    /**
     * Status of this sensor, in string form
     *
     * @return status
     */
    @Override
    public String status() {
        String functionName = "status";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return null;
    }

    /**
     * Change to calibration or measurement mode
     *
     * @param mode
     */
    @Override
    public void setMode(CompassMode mode) {
        String functionName = "setMode";

        if (DEBUG) {
            RobotLog.d("%s(%s)", functionName, mode.toString());
        }

        // TODO: IMPLEMENT THIS

//        this.writeCommand(mode==CompassMode.CALIBRATION_MODE ? Command.CALIBRATE_IRON : Command.NORMAL);

    }

    /**
     * Check to see whether calibration was successful.
     * After attempting a calibration, the hardware will (eventually) indicate whether or
     * not it was unsuccessful. The default is "success", even when the calibration is not
     * guaranteed to have completed successfully.
     * <p>
     * A user should monitor this field for (at least) several seconds to determine success.
     *
     * @return failure
     */
    @Override
    public boolean calibrationFailed() {
        String functionName = "calibrationFailed";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        // TODO: IMPLEMENT THIS

        return false;
    }

    // Test to see if valid data is being returned from the device
    public boolean testConnection() {
        boolean test = false;
        char[] testChars = new char[3];

        //byte[] data = this.deviceClient.read(Register.HMC5883L_RA_ID_A.bVal, 3);
        testChars[0] = (char)this.deviceClient.read8(Register.HMC5883L_RA_ID_A.bVal);
        testChars[1] = (char)this.deviceClient.read8(Register.HMC5883L_RA_ID_B.bVal);
        testChars[2] = (char)this.deviceClient.read8(Register.HMC5883L_RA_ID_B.bVal);

//        for (int i = 0; i < 3; i++) {
//            testChars[i] = (char)data[i];
//        }

        // Should see the characters 'H', '4' and '3' in the registers.
        if ((testChars[0] == 'H') && (testChars[1] == '4') && (testChars[2] == '3')) {
            test = true;
            RobotLog.d("testConnection: true");
        }
        RobotLog.d("%c%c%c",testChars[0], testChars[1], testChars[2]);

        return test;
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        String functionName = "getManufacturer";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        String functionName = "getDeviceName";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return DEVICE_NAME;
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public byte read8(Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }

    public void write8(Register reg, byte value)
    {
        this.write8(reg, value, I2cWaitControl.ATOMIC);
    }
    public void write8(Register reg, byte value, I2cWaitControl control)
    {
        this.deviceClient.write8(reg.bVal, value, control);
    }

    public int readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(this.deviceClient.read(reg.bVal, 2), ByteOrder.LITTLE_ENDIAN);
    }

    public void writeShort(Register reg, short value)
    {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value, ByteOrder.LITTLE_ENDIAN));
    }

//    public void writeCommand(Command command)
//    {
//        this.write8(Register.COMMAND, command.bVal, true);
//    }
//
//    public Command readCommand()
//    {
//        return Command.fromByte(this.read8(Register.COMMAND));
//    }

    //
    // Compass Functions
    //

    //
    // CONFIG_A Register
    //
    public byte getSampleAveraging() {
        String functionName = "getSampleAveraging";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }
        byte retVal = read8(Register.HMC5883L_RA_CONFIG_A);

        retVal &= 0x60;

        retVal >>= 0x05;

        return retVal;
    }

    public void setSampleAveraging(byte averaging) {
        String functionName = "setSampleAveraging";

        if (DEBUG) {
            RobotLog.d("%s(%d)", functionName, averaging);
        }


    }

    public byte getDataRate() {
        String functionName = "getDataRate";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte retVal = read8(Register.HMC5883L_RA_CONFIG_A);

        retVal &= 0x1c;
        retVal >>= 3;

        return retVal;
    }

    public void setDataRate(byte rate) {
        String functionName = "setDataRate";

        if (DEBUG) {
            RobotLog.d("%s(%d)", functionName, rate);
        }

    }

    public byte getMeasurementBias() {
        String functionName = "getMeasurementBias";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }
        byte retVal = read8(Register.HMC5883L_RA_CONFIG_A);

        retVal &= 0x03; // Mask the top six bits

        return retVal;
    }

    public void setMeasurementBias(byte bias) {
        String functionName = "setMeasurementBias";

        if (DEBUG) {
            RobotLog.d("%s(%d)", functionName, bias);
        }


    }


    //
    // CONFIG_B Register
    //

    // Get the gain setting
    /*
     * Value | Field Range | Gain (LSB/Gauss)
     * ------+-------------+-----------------
     * 0     | +/- 0.88 Ga | 1370
     * 1     | +/- 1.3 Ga  | 1090 (Default)
     * 2     | +/- 1.9 Ga  | 820
     * 3     | +/- 2.5 Ga  | 660
     * 4     | +/- 4.0 Ga  | 440
     * 5     | +/- 4.7 Ga  | 390
     * 6     | +/- 5.6 Ga  | 330
     * 7     | +/- 8.1 Ga  | 230
     */
    public byte getGain() {
        String functionName = "getGain";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte retVal = read8(Register.HMC5883L_RA_CONFIG_B);

        retVal &= 0x07;  // use mask to clear top 5 bits

        return retVal;
    }

    public void setGain(byte gain) {
        String functionName = "setGain";

        if (DEBUG) {
            RobotLog.d("%s(%d)", functionName, gain);
        }


    }


    //
    // Mode Register
    //
    public byte getMode() {
        String functionName = "getMode";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte retVal = read8(Register.HMC5883L_RA_STATUS);
        retVal &= 0x03; // Clear the upper six bits

        return retVal;
    }


    public void setMode(byte mode) {
        String functionName = "setMode";

        if (DEBUG) {
            RobotLog.d("%s(%d)", functionName, mode);
        }

    }

    //
    // Data* Registers
    //
    public short[] getHeading() {
        String functionName = "getHeading";
        short[] returnVals = new short[3];

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }
        byte[] values = this.deviceClient.read(Register.HMC5883L_RA_DATAX_H.bVal, 6);

        returnVals[0] = (short)((values[0] << 8) | values[1]);
        returnVals[1] = (short)((values[2] << 8) | values[3]);
        returnVals[2] = (short)((values[4] << 8) | values[5]);

        // returns x,y,z headings as short array
        return returnVals;
    }

    // Get the X heading
    // While this look inefficient, the chip requires that the full buffer of six bytes containing
    // the X, Y and Z data is retrieved at the same time
    public short getHeadingX() {
        String functionName = "getHeadingX";
        short[] returnVals = new short[3];

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte[] values = this.deviceClient.read(Register.HMC5883L_RA_DATAX_H.bVal, 6);

        returnVals[0] = (short)((values[0] << 8) | values[1]);
//        returnVals[1] = (short)((values[2] << 8) | values[3]);
//        returnVals[2] = (short)((values[4] << 8) | values[5]);

        return returnVals[0];
    }

    // Get the Y heading
    // While this look inefficient, the chip requires that the full buffer of six bytes containing
    // the X, Y and Z data is retrieved at the same time
    public short getHeadingY() {
        String functionName = "getHeadingY";
        short[] returnVals = new short[3];

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte[] values = this.deviceClient.read(Register.HMC5883L_RA_DATAX_H.bVal, 6);

//        returnVals[0] = (short)((values[0] << 8) | values[1]);
        returnVals[1] = (short)((values[2] << 8) | values[3]);
//        returnVals[2] = (short)((values[4] << 8) | values[5]);

        return returnVals[1];
    }

    // Get the Z heading
    // While this look inefficient, the chip requires that the full buffer of six bytes containing
    // the X, Y and Z data is retrieved at the same time
    public short getHeadingZ() {
        String functionName = "getHeadingZ";
        short[] returnVals = new short[3];

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte[] values = this.deviceClient.read(Register.HMC5883L_RA_DATAX_H.bVal, 6);

//        returnVals[0] = (short)((values[0] << 8) | values[1]);
//        returnVals[1] = (short)((values[2] << 8) | values[3]);
        returnVals[2] = (short)((values[4] << 8) | values[5]);

        return returnVals[1];
    }

    //
    // Status Register
    //

    public boolean getLockStatus() {
        String functionName = "getLockStatus";
        boolean status = false;

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte response = read8(Register.HMC5883L_RA_STATUS);
        if ((response & (1 << (HMC5883L_STATUS_LOCK_BIT - 1))) > 0) {
            status = true;
        }

        return status;
    }

    public boolean getReadyStatus() {
        String functionName = "getReadyStatus";
        boolean status = false;

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        byte response = read8(Register.HMC5883L_RA_STATUS);
        if ((response & (1 << (HMC5883L_STATUS_READY_BIT - 1))) > 0) {
            status = true;
        }

        return status;
    }

    //
    // ID* registers
    //
    public byte getIDA() {
        String functionName = "getIDA";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return read8(Register.HMC5883L_RA_ID_A);
    }

    public byte getIDB() {
        String functionName = "getIDB";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return read8(Register.HMC5883L_RA_ID_B);
    }

    public byte getIDC() {
        String functionName = "getIDC";

        if (DEBUG) {
            RobotLog.d("%s", functionName);
        }

        return read8(Register.HMC5883L_RA_ID_C);
    }

}
