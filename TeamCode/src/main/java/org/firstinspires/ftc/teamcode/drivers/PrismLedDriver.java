package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * The goBILDA Prism RGB LED Driver (SKU 3118-2855-0001) as an I2C device the robot
 * configuration knows about. Pick "goBILDA Prism LED Driver" on an I2C port and name it.
 *
 * <p>This class only moves bytes. The Prism's user guide (November 2025) says: I2C address
 * 0x38; register 0x00 is a device id; 0x05 a 4-byte status; registers 0x08 to 0x11 are ten
 * animation layers, and a write to a layer is a sub-register byte followed by its data.
 * What those bytes mean lives in {@code subsystems/LightStrip}.
 *
 * <p>Written the way the SDK's own GoBildaPinpointDriver is written: the two annotations
 * put it in the configuration list, and the SDK builds it with the constructor below.
 */
@I2cDeviceType
@DeviceProperties(
        name = "goBILDA Prism LED Driver",
        xmlTag = "goBILDAPrismLedDriver",
        description = "goBILDA Prism RGB LED Driver over I2C")
public class PrismLedDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    /** The Prism's I2C address, from its user guide. */
    public static final int I2C_ADDRESS = 0x38;
    public static final int REG_DEVICE_ID = 0x00;
    public static final int REG_STATUS = 0x05;
    public static final int REG_LAYER_0 = 0x08;
    public static final int LAYERS = 10;
    // ---- Sub-registers every animation has, from the user guide ----
    public static final int SUB_ANIMATION = 0x00;
    public static final int SUB_BRIGHTNESS = 0x01;
    public static final int SUB_START = 0x02;
    public static final int SUB_STOP = 0x03;
    public static final int SUB_PRIMARY_COLOR = 0x04;
    public static final int SUB_SECONDARY_COLOR = 0x05;
    public static final int SUB_PERIOD = 0x06;
    public static final int SUB_PRIMARY_PERIOD = 0x07;
    // ---- Animations. 0 and 1 are from the guide's example. CHECK the rest on the strip:
    // they are numbered here in the order the guide describes them. ----
    public static final int ANIMATION_NONE = 0;
    public static final int ANIMATION_SOLID = 1;
    public static final int ANIMATION_BLINK = 2;
    public static final int ANIMATION_PULSE = 3;
    public static final int ANIMATION_RAINBOW = 6;

    public PrismLedDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA Prism LED Driver";
    }

    /** The device id register, 0 to 255. */
    public int ReadDeviceId() {
        return deviceClient.read8(REG_DEVICE_ID) & 0xFF;
    }

    /** The 4-byte status register, as the Prism sent it. */
    public byte[] ReadStatus() {
        return deviceClient.read(REG_STATUS, 4);
    }

    /**
     * Paint one color on a run of LEDs, on one layer. Start is the first LED, counting from
     * 0 at the Prism; stop is where the run ends. CHECK on the strip: the guide's example
     * uses start 0, stop 10 for the first ten LEDs, so stop is taken as "one past the last".
     * If each run comes out one LED too long, stop is the last LED itself.
     *
     * @param layer      0 to 9. Higher layers draw on top of lower ones.
     * @param brightness 0 to 100
     * @param red        0 to 255, likewise green and blue
     */
    public void SolidColor(int layer, int startLed, int stopLed, int brightness, int red, int green, int blue) {
        WriteLayer(layer, SUB_ANIMATION, new byte[] {(byte) ANIMATION_SOLID});
        WriteLayer(layer, SUB_BRIGHTNESS, new byte[] {(byte) brightness});
        WriteLayer(layer, SUB_START, new byte[] {(byte) startLed});
        WriteLayer(layer, SUB_STOP, new byte[] {(byte) stopLed});
        WriteLayer(layer, SUB_PRIMARY_COLOR, new byte[] {(byte) red, (byte) green, (byte) blue});
    }

    /** Nothing on this layer. LEDs no layer paints stay dark. */
    public void ClearLayer(int layer) {
        WriteLayer(layer, SUB_ANIMATION, new byte[] {(byte) ANIMATION_NONE});
    }

    /**
     * One message to a layer: the sub-register, then its bytes.
     *
     * @param layer       0 to 9
     * @param subRegister which setting of the animation, from the user guide
     * @param data        the setting's bytes
     */
    public void WriteLayer(int layer, int subRegister, byte[] data) {
        byte[] message = new byte[data.length + 1];
        message[0] = (byte) subRegister;
        for (int i = 0; i < data.length; i++) {
            message[i + 1] = data[i];
        }
        deviceClient.write(REG_LAYER_0 + layer, message);
        // Wait until it is sent. Without this, a quick run of writes can replace each other
        // before they leave the hub, and the later layers never reach the strip.
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }
}
