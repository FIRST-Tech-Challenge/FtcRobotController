package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;


@TeleOp
public class ColorSensorConfig extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> blue
        only pin1 --> red
        neither   --> no object
         */
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 180 / 360.0 * 255, 250 / 360.0 * 255); // blue
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 75 / 360.0 * 255); // yellow
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 20); // 20mm or closer requirement

        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 140 / 360.0 * 255, 210 / 360.0 * 255); // inverted red
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 235 / 360.0 * 255, 255 / 360.0 * 255); // inverted yellow
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 20); // 20mm or closer requirement
        crf.setPin1InvertHue(); // invert hue values

        waitForStart();

        stop();
    }
}

/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class ColorRangefinder {
    public final RevColorSensorV3 emulator;
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this.emulator = emulator;
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    public void setI2cAddress(int value) {
        i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    /**
     * Read distance via I2C
     * @return distance in millimeters
     */
    public double readDistance() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
        return ((hue360 - 180) % 360);
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}

