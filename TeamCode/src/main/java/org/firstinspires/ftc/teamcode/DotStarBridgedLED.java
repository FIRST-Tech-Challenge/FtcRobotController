package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.Arrays;

/**
 * Represents a DotStar LED strip when plugged in via an I2C/SPI bridge.
 *
 * DotStar LEDs (i.e. https://www.adafruit.com/product/2238) are collections of LEDs which are
 * programmable using SPI. While it is possible to use two digital outputs as data and clock lines,
 * an I2C/SPI Bridge can manage the digital writes at a much higher frequency. This is required for
 * "smooth" color changes. However, a similar class for driving the LEDs via two digital outputs
 * is also available.
 *
 * Output intensity is artificially limited by the theoretical maximum allowed by the digital IO
 * controller {@link DotStarBridgedLED#setMaxOutputAmps(double)}. Be aware that exceeding the
 * allowed current can damage your devices. It is your responsibility to ensure this doesn't happen.
 *
 * If using the REV Robotics Expansion Hub to run the I2C/SPI bridge, please ensure you have
 * firmware version 1.7.2 or greater. Otherwise, the heavy I2C write load may cause crashes. Also,
 * do not use I2C bus/port 0, as the operation of the bridge will interfere with the internal IMU.
 *
 * @author AJ Foster and Mike Nicolai
 * @version 2.0.0
 */
@SuppressWarnings({"unused", "WeakerAccess"})
@DeviceProperties(name = "DotStar LEDs via SPI Bridge", description = "DotStar LED strip connected via an I2C/SPI bridge", xmlTag = "DotStarBridgedLED")
@I2cDeviceType()
public class DotStarBridgedLED extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynchSimple, DotStarBridgedLED.Parameters> {

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    /**
     * Array representing the individual pixel groups in the LED strip.
     *
     * Sizing can be set using {@link Parameters#length} during initialization.
     * */
    public DotStarBridgedLED.Pixel[] pixels;

    /**
     * Array representing the individual pixel groups in the LED strip as set.
     *
     * Sizing can be set using {@link Parameters#length} during initialization.
     * */
    private DotStarBridgedLED.Pixel[] setPixels;


    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public DotStarBridgedLED(I2cDeviceSynch deviceClient) {
        this(new Parameters(), deviceClient, true);
    }

    public DotStarBridgedLED(DotStarBridgedLED.Parameters params, I2cDeviceSynchSimple deviceClient, boolean isOwned) {
        super(deviceClient, isOwned, params);

        // Pass logging parameters through to device client.
        this.deviceClient.setLogging(this.parameters.loggingEnabled);
        this.deviceClient.setLoggingTag(this.parameters.loggingTag);

        // Create array for pixels.
        this.pixels = new DotStarBridgedLED.Pixel[params.length];
        this.setPixels = new DotStarBridgedLED.Pixel[params.length];

        for (int i = 0; i < params.length; i++) {
            pixels[i] = new DotStarBridgedLED.Pixel(0, 0, 0);
            // Set to invalid value so first time through it will get udpated.
            setPixels[i] = new DotStarBridgedLED.Pixel(-1, -1, -1);
        }

        // We ask for an initial callback here; that will eventually call internalInitialize().
        this.registerArmingStateCallback(true);
        this.engage();
    }


    //----------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------

    // Can be called publicly via #initialize(Parameters).
    @Override
    protected synchronized boolean internalInitialize(Parameters parameters) {
        this.parameters = parameters.clone();
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        return true;
    }


    //----------------------------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------------------------

    /** Reset each pixel in the strip to "off". */
    public void clear() {
        for (DotStarBridgedLED.Pixel pixel : pixels) {
            pixel.reset();
        }
    }

    /**
     * Set the pixel at index with the given color values.
     *
     * @param index Index of the pixel to set
     * @param red   Red color value
     * @param green Green color value
     * @param blue  Blue color value
     */
    public void setPixel(int index, int red, int green, int blue) {
        this.pixels[index].setRGB(red, green, blue);
    }

    /**
     * Set the pixel at index with the given color values.
     *
     * @param index Index of the pixel to set
     * @param color Color value (android.graphics.Color)
     *
     * @see android.graphics.Color
     */
    public void setPixel(int index, int color) {
        this.pixels[index].setColor(color);
    }

    /**
     * Get the length of the LED strip as currently set.
     *
     * @return Current setting of the LED strip's length.
     */
    public int getLength() {
        return this.pixels.length;
    }

    /**
     * Set the length of the LED strip.
     *
     * @param length Number of pixels to make available.
     */
    public void setLength(int length) {
        if (length < 1) {
            throw new IllegalArgumentException("LED strip length must be at least 1");
        }

        int oldLength = this.pixels.length;
        this.parameters.length = length;

        if (oldLength == length) {
            return;
        }
        else if (oldLength > length) {
            this.pixels = Arrays.copyOfRange(this.pixels, 0, length);
            this.setPixels = Arrays.copyOfRange(this.setPixels, 0, length);
        }
        else {
            this.pixels = Arrays.copyOf(this.pixels, length);
            this.setPixels = Arrays.copyOf(this.setPixels, length);

            for (int i = oldLength; i < length; i++) {
          		this.pixels[i] = new DotStarBridgedLED.Pixel(0, 0, 0);
           		this.setPixels[i] = new DotStarBridgedLED.Pixel(-1, -1, -1);
            }
        }
    }

    /**
     * Set the type of controller used to drive the LED strip.
     *
     * This will set the i2cMaxBuffer and maxOutputAmps settings to appropriate values for the
     * given controller. These values can also be set manually.
     *
     * @param controller Controller in use.
     */
    public void setController(Controller controller) {
        this.parameters.setController(controller);
    }

    /**
     * Returns the i2cMaxBuffer setting currently in effect.
     *
     * @return Maximum number of bytes to be written to the I2C bus at once.
     */
    public int getI2cMaxBuffer() {
        return this.parameters.i2cMaxBuffer;
    }

    /**
     * Set the maximum number of bytes to be written to the I2C bus at once.
     *
     * @param buffer Maximum bytes to write
     */
    public void setI2cMaxBuffer(int buffer) {
        if (buffer < 1) {
            throw new IllegalArgumentException("I2C buffer size must be at least 1");
        }

        this.parameters.i2cMaxBuffer = buffer;
    }

    /**
     * Returns the maxOutputAmps setting currently in effect.
     *
     * @return Maximum current allowed to flow to the LEDs.
     */
    public double getMaxOutputAmps() {
        return this.parameters.maxOutputAmps;
    }

    /**
     * Set the maximum current allowed to flow to the LEDs.
     *
     * @param amps Maximum amps to draw
     */
    public void setMaxOutputAmps(double amps) {
        if (amps < 0.02) {
            throw new IllegalArgumentException("A minimum of 0.02 amps are required to run LEDs");
        }
        else if (amps > 10.0) {
            throw new IllegalArgumentException("An excessively high maximum amperage is dangerous");
        }

        this.parameters.maxOutputAmps = amps;
    }

    /**
     * Flush the current array of pixels to the device.
     *
     * We attempt to perform this update in a careful manner. Total brightness of the pixels will
     * be reduced (possibly reducing color quality) if the projected current drawn by the pixels
     * exceeds the maxOutputAmps setting.
     *
     * @see DotStarBridgedLED#setMaxOutputAmps(double)
     * */
    public void update() {
        boolean updatePixels = false;
        // Number of bytes necessary to write out the pixels, including header and end frames.
        int bufferLength =
                4                               // Header frame: 1 word of zeroes
                        + 4 * pixels.length             // Each pixel: 1 word
                        + (pixels.length + 15) / 16;    // End frame: 1 byte for every 16 pixels.

        // This will be written out to the I2C device once filled.
        byte[] buffer = new byte[bufferLength];

        // Temporarily holds just the colors from the pixels.
        int[] colors = new int[pixels.length * 3];

        // Used to track the total expected current drawn.
        double current = 0.0;

        // Iterate the pixels to learn the total current drawn and collect the colors in order.
        for (int i = 0; i < pixels.length; i++) {
            if(!pixels[i].equals(setPixels[i])) {
                setPixels[i].copy(pixels[i]);
                updatePixels = true;
            }
            Pixel pixel = pixels[i];
            current += pixel.current();

            // When written out to the strip, we'll need the colors in BGR order.
            colors[i * 3] = pixel.blue;
            colors[i * 3 + 1] = pixel.green;
            colors[i * 3 + 2] = pixel.red;
        }

        // Only perform the write if one or more pixels changed.  Otherwise
        // the pixels should already by set properly.
        if(updatePixels) {
            // Ensure the total current will not exceed our theoretical maximum.
            if (current > parameters.maxOutputAmps) {
                double scale = parameters.maxOutputAmps / current;

                // Scale (reduce) each color value (0 - 255) and round down.
                for (int i = 0; i < colors.length; i++) {
                    colors[i] = (int) Math.floor(colors[i] * scale);
                }
            }

            // Skip the first four bytes (header frame).
            for (int i = 0, j = 0; i < bufferLength; i++) {

                if (i < 4) {
                    buffer[i] = 0;
                    continue;
                }

                // NOTE: Writing zeroes instead of 0xff reduces odd end pixels if not
                // using the entire strip. This isn't to spec, however.
                // If we have written all of our colors, fill in the end frame with 0x00.
                if (j >= colors.length) {
                    buffer[i] = (byte) 0x00;
                    continue;
                }

                // We still have colors to write, so write {0xff, blue, green, red}.
                if (i % 4 == 0) {
                    buffer[i] = (byte) 0xff;
                    continue;
                }

                // As we write colors, progress along the colors array.
                buffer[i] = (byte) colors[j];
                j++;
            }

            // Write to the LED strip.
            write(buffer);
        }
    }


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /**
     * Writes out the given buffer to the LEDs via the I2C/SPI bridge.
     *
     * This method will attempt to write the given data in the most efficient way possible based on
     * on the parameters.i2cMaxBuffer. In the worst case, bytes will be written one at a time. The
     * destination register is determined by parameters.writeRegister. Atomic write waiting is used
     * to guarantee that writes are made successfully (though not necessarily completed by the time
     * this method returns).
     *
     * @param buffer Raw data to write out, including frame boundaries and termination bytes.
     */
    protected void write(byte[] buffer) {

        // Write the largest "chunks" possible for the I2C bus.
        if (parameters.i2cMaxBuffer > 1) {
            int left = 0, right;

            // Use left, right to track the bounds of the next chunk to write.
            while (left < buffer.length) {
                // Minus one, because the register counts as a byte.
                right = Math.min(left + parameters.i2cMaxBuffer - 1, buffer.length);

                this.deviceClient.write(
                        this.parameters.writeRegister,
                        Arrays.copyOfRange(buffer, left, right),
                        I2cWaitControl.WRITTEN
                );

                left = right;
            }
        }

        // If necessary, write one byte at a time.
        else {
            for (byte item : buffer) {
                this.deviceClient.write8(this.parameters.writeRegister, item);
                this.deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
            }
        }
    }


    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public String getDeviceName() {
        return "DotStar LED via I2C/SPI Bridge";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }


    //----------------------------------------------------------------------------------------------
    // Parameters
    //----------------------------------------------------------------------------------------------

    /**
     * Instances of Parameters contain data indicating how the LED strip is to be initialized.
     */
    public static class Parameters implements Cloneable {

        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        /** The write address of the I2C/SPI bridge. (Default: 0x50) */
        public I2cAddr i2cAddr = I2cAddr.create8bit(0x50);

        /** Maximum size of the I2C buffer as determined by the hardware. (Default: 27 bytes) */
        public int i2cMaxBuffer = 27; // Default to Modern Robotics Core DIM (lowest known).

        /** Number of pixels present in the LED strip. (Default: 30) */
        public int length = 30;

        /** Whether to log the actions of this device. (Default: no) */
        public boolean loggingEnabled = false;

        /** Label to use when logging the actions of this device. (Default: DotStarBridgedLED) */
        public String loggingTag = "DotStarBridgedLED";

        /** Maximum output current (in amps) as determined by the hardware. (Default: 0.2 amps) */
        public double maxOutputAmps = 0.2; // Default to Modern Robotics Core DIM (lowest known).

        /** Bridge "register" (buffer prefix) to pass-through to the LEDs. (Default: 0x01) */
        public int writeRegister = 0x01;


        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        public Parameters() {}

        public Parameters(Controller controller) {
            setController(controller);
        }

        public Parameters clone() {
            try {
                return (Parameters) super.clone();
            }
            catch (CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal error: DotStarBridgedLED.Parameters not cloneable");
            }
        }

        public void setController(Controller controller) {
            switch (controller) {
                case RevExpansionHub:
                    this.i2cMaxBuffer = 100;
                    this.maxOutputAmps = 1.5;
                    break;

                case ModernRoboticsDIM:
                default:
                    this.i2cMaxBuffer = 27;
                    this.maxOutputAmps = 0.2;
                    break;
            }
        }
    }


    //----------------------------------------------------------------------------------------------
    // Pixels
    //----------------------------------------------------------------------------------------------

    /**
     * Pixels represent a single pixel group in the strip of LEDs, with red, green, and blue values.
     */
    public static class Pixel {

        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        /** Value of the red channel, from 0 to 255. */
        private int red;

        /** Value of the blue channel, from 0 to 255. */
        private int blue;

        /** Value of the green channel, from 0 to 255. */
        private int green;

        /** Estimate of maximum amps drawn per color. */
        public static final double ampsDrawn = 0.02;


        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        public Pixel() {
            reset();
        }

        public Pixel(int red, int green, int blue) {
            this.red = bound(red);
            this.blue = bound(blue);
            this.green = bound(green);
        }


        //------------------------------------------------------------------------------------------
        // Public API
        //------------------------------------------------------------------------------------------

        /**
         * Gives an estimate of the amount of current (in amps) needed to display the pixel.
         *
         * @return Estimated current drawn (in amps).
         */
        public double current() {
            return (red + blue + green) / 255.0 * ampsDrawn;
        }


        /**
         * Resets the pixel to "off" with values (0, 0, 0).
         */
        public void reset() {
            this.red = 0;
            this.blue = 0;
            this.green = 0;
        }


        /**
         * Set the pixel's color values (R, G, B).
         *
         * @param red   Red color value.
         * @param green Green color value.
         * @param blue  Blue color value.
         */
        public void setRGB(int red, int green, int blue) {
            this.red = bound(red);
            this.blue = bound(blue);
            this.green = bound(green);
        }

        /**
         * Set the pixel's color values (android.graphics.Color).
         *
         * @param color Color value (android.graphics.Color)
         *
         * @see android.graphics.Color
         */
        public void setColor(int color) {
            this.red = bound(Color.red(color));
            this.blue = bound(Color.blue(color));
            this.green = bound(Color.green(color));
        }

        /**
         * Copies a pixel to another.
         * @param c The pixel to copy.
         */
        public void copy(Pixel c) {
            red = c.red;
            green = c.green;
            blue = c.blue;
        }

        /**
         * Overriding equals() to compare two pixels
         *
         * @param o Object to compare to this
         * @return true if pixels are equal, false if different
         */
        @Override
        public boolean equals(Object o) {
            // If the object is compared with itself then return true
            if (o == this) {
                return true;
            }

        /* Check if o is an instance of Pixel or not
          "null instanceof [type]" also returns false */
            if (!(o instanceof Pixel)) {
                return false;
            }

            // typecast o to Pixel so that we can compare data members
            Pixel c = (Pixel) o;

            // Compare the data members and return accordingly
            return ((red == c.red) && (green == c.green) && (blue == c.blue));
        }

        //------------------------------------------------------------------------------------------
        // Utility
        //------------------------------------------------------------------------------------------

        /**
         * Returns the given value, clipped to the range 0 - 255.
         *
         * @param value Color value to clip.
         * @return      Color value clipped to the nearest value in the range 0 - 255.
         */
        private int bound(int value) {
            return Math.max(0, Math.min(255, value));
        }
    }


    /**
     * Used to control settings such as the maximum I2C buffer size and maximum current available.
     */
    public enum Controller {
        ModernRoboticsDIM,
        RevExpansionHub,
        Other
    }
}
