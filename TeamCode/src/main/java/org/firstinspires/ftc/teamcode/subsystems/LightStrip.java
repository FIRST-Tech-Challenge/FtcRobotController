package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.drivers.PrismLedDriver;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

/**
 * The goBILDA Prism RGB LED driver and the LED strip plugged into it, over I2C.
 *
 * <p>The Prism does the animating. We send it short messages: "layer 0, animation 1 (solid),
 * color red". It keeps showing that until told otherwise, so nothing is sent every loop.
 * Everything here comes from goBILDA's user guide for SKU 3118-2855-0001 (Nov 2025): each
 * of the Prism's ten layers has sub-registers (0x00 which animation, 0x01 brightness, 0x02
 * start pixel, 0x03 stop pixel, 0x04 color). {@link PrismLedDriver} moves the bytes; it is
 * what the robot configuration calls "goBILDA Prism LED Driver".
 *
 * <p>Controls, for {@code Test: Light Strip} and HyDrive (the D-pad is free):
 * <ul>
 *   <li><b>D-pad up</b> red, <b>right</b> green, <b>down</b> blue, all solid.
 *   <li><b>D-pad left</b>: off.
 * </ul>
 *
 * <p>Two things the guide does not pin down, marked CHECK below: which number each
 * animation after Solid Color has, and the byte order of 4-byte values like a blink period.
 * The first test on the strip settles both. See docs/decisions/0017.
 */
public class LightStrip implements Subsystem {
    /** The one layer used so far. The Prism has ten, 0 to 9, drawn on top of each other. */
    static final int LAYER = 0;

    private final HydraOpMode mOp;
    private final BotProperties mProperties;
    private final Gamepad mGamepad;
    /** null when no strip is in the robot configuration. */
    private final PrismLedDriver mStrip;
    private final Debouncer mRedButton;
    private final Debouncer mGreenButton;
    private final Debouncer mBlueButton;
    private final Debouncer mOffButton;

    private boolean mStarted = false;
    private int mDeviceId = -1;
    private byte[] mStatus = new byte[0];
    private String mShowing = "nothing sent yet";

    public LightStrip(HydraOpMode op, BotProperties properties, Gamepad gamepad) {
        mOp = op;
        mProperties = properties;
        mGamepad = gamepad;
        mStrip = op.mHardwareMap.tryGet(PrismLedDriver.class, properties.cfgStrip);
        mRedButton = new Debouncer(properties.debounce);
        mGreenButton = new Debouncer(properties.debounce);
        mBlueButton = new Debouncer(properties.debounce);
        mOffButton = new Debouncer(properties.debounce);
    }

    /** Read who is on the other end, once, so the Driver Station can show it. */
    @Override
    public boolean Init() {
        if (mStrip == null) {
            mOp.mTelemetry.addLine("No light strip: nothing named '" + mProperties.cfgStrip
                    + "' in the robot configuration (a goBILDA Prism LED Driver).");
            return true;
        }
        if (!mStarted) {
            mStarted = true;
            mDeviceId = mStrip.ReadDeviceId();
            mStatus = mStrip.ReadStatus();
        }
        ShowStatus();
        return true;
    }

    /** D-pad: up red, right green, down blue, left off. Each press is one message. */
    @Override
    public void HandleUserInput() {
        mRedButton.In(mGamepad.dpad_up);
        if (mRedButton.Out()) {
            mRedButton.Used();
            Solid(255, 0, 0);
        }
        mGreenButton.In(mGamepad.dpad_right);
        if (mGreenButton.Out()) {
            mGreenButton.Used();
            Solid(0, 255, 0);
        }
        mBlueButton.In(mGamepad.dpad_down);
        if (mBlueButton.Out()) {
            mBlueButton.Used();
            Solid(0, 0, 255);
        }
        mOffButton.In(mGamepad.dpad_left);
        if (mOffButton.Out()) {
            mOffButton.Used();
            Off();
        }
    }

    @Override
    public void Process() {
        ShowStatus();
    }

    /** The whole strip one color. Red, green and blue are 0 to 255. */
    public void Solid(int red, int green, int blue) {
        if (mStrip != null) {
            mStrip.SolidColor(LAYER, 0, mProperties.stripLedCount, mProperties.stripBrightness, red, green, blue);
        }
        mShowing = "solid " + red + "," + green + "," + blue;
    }

    /**
     * The whole strip blinking between two colors. CHECK: ANIMATION_BLINK's number and the
     * byte order of the period are the guide's best reading, not yet seen on a strip.
     */
    public void Blink(int red, int green, int blue, int red2, int green2, int blue2, long periodMs) {
        WriteLayer(PrismLedDriver.SUB_ANIMATION, new byte[] {(byte) PrismLedDriver.ANIMATION_BLINK});
        WriteLayer(PrismLedDriver.SUB_BRIGHTNESS, new byte[] {(byte) mProperties.stripBrightness});
        WriteLayer(PrismLedDriver.SUB_START, new byte[] {0});
        WriteLayer(PrismLedDriver.SUB_STOP, new byte[] {(byte) mProperties.stripLedCount});
        WriteLayer(PrismLedDriver.SUB_PRIMARY_COLOR, new byte[] {(byte) red, (byte) green, (byte) blue});
        WriteLayer(PrismLedDriver.SUB_SECONDARY_COLOR, new byte[] {(byte) red2, (byte) green2, (byte) blue2});
        WriteLayer(PrismLedDriver.SUB_PERIOD, Bytes32(periodMs));
        WriteLayer(PrismLedDriver.SUB_PRIMARY_PERIOD, Bytes32(periodMs / 2));
        mShowing = "blink " + red + "," + green + "," + blue + " and " + red2 + "," + green2 + "," + blue2
                + " every " + periodMs + " ms";
    }

    /** No animation on our layer: the strip goes dark. */
    public void Off() {
        if (mStrip != null) {
            mStrip.ClearLayer(LAYER);
        }
        mShowing = "off";
    }

    /** One setting to our layer, if there is a strip. */
    private void WriteLayer(int subRegister, byte[] data) {
        if (mStrip == null) {
            return;
        }
        mStrip.WriteLayer(LAYER, subRegister, data);
    }

    /**
     * A 4-byte value, lowest byte first (little-endian), which is how most small chips store
     * numbers. CHECK on the strip: if a 2000 ms blink comes out wildly wrong, flip this order.
     */
    static byte[] Bytes32(long value) {
        byte[] bytes = new byte[4];
        bytes[0] = (byte) (value & 0xFF);
        bytes[1] = (byte) ((value >> 8) & 0xFF);
        bytes[2] = (byte) ((value >> 16) & 0xFF);
        bytes[3] = (byte) ((value >> 24) & 0xFF);
        return bytes;
    }

    /** A byte as 0 to 255 instead of -128 to 127. */
    static int Unsigned(byte b) {
        return b & 0xFF;
    }

    public void ShowStatus() {
        if (mStrip == null) {
            return;
        }
        mOp.mTelemetry.addData("Light strip", "showing " + mShowing);
        mOp.mTelemetry.addData("Strip device id", "0x%02X (register 0x00)", mDeviceId);
        if (mStatus.length == 4) {
            // Guide: bits 0 to 7 of the status register are the number of LEDs, 8 to 23 the FPS.
            // Lowest byte first means byte 0 is the LED count.
            mOp.mTelemetry.addData("Strip status bytes", "%02X %02X %02X %02X (LEDs %d if lowest byte first)",
                    Unsigned(mStatus[0]), Unsigned(mStatus[1]), Unsigned(mStatus[2]), Unsigned(mStatus[3]),
                    Unsigned(mStatus[0]));
        }
        mOp.mTelemetry.addLine("D-pad: up red, right green, down blue, left off");
    }
}
