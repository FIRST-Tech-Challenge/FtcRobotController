package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Represents a DotStar LED strip when plugged in to two digital IO channels.
 *
 * DotStar LEDs (i.e. https://www.adafruit.com/product/2238) are collections of LEDs which are
 * programmable using two digital output wires: one for data, and one as a clock line. Unlike
 * NeoPixels, DotStar LEDs do not require the controller to operate at a particular speed. Instead,
 * turning the clock line's output off and on acts as a signal that the LED's microcontroller
 * should read the next bit from the data line.
 *
 * Using two digital IO pins in this way gives "just okay" performance, as the refresh rate of the
 * pixels is limited by how fast the digital IO controller can switch the clock line on and off.
 * For truly awesome performance, the LEDs can be connected to an I2C port using an I2C/SPI bridge
 * module (subject to the legality of the bridge). A similar class exists to enable use of bridged
 * LEDs (DotStarBridgedLED).
 *
 * Output intensity is artificially limited by the theoretical maximum allowed by the digital IO
 * controller {@link #setMaxOutputAmps(double)}. Be aware that exceeding the allowed current can
 * damage your devices. It is your responsibility to ensure this doesn't happen.
 *
 * @author AJ Foster and Mike Nicolai
 * @version 1.0.1
 */
public class DotStarLED {

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    /** Array of pixels that will be written to the LEDs on {@link #update()}. */
    public DotStarLED.Pixel[] pixels;

    /** Digital IO channels used for the clock and data lines. */
    private DigitalChannel clock, data;

    /** Length of the LED strip (number of pixels). */
    private int length;

    /** Maximum output current (in amps) as determined by the hardware. */
    private double maxOutputAmps = 0.2; // Default to Modern Robotics Core DIM (lowest known).

    /** Last set of colors written to the LEDs. */
    private int[] colors;


    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     *
     * @param length    Number of LEDs present in the strip.
     * @param clock     Digital IO channel for the clock line.
     * @param data      Digital IO channel for the data line.
     */
    public DotStarLED(int length, DigitalChannel clock, DigitalChannel data) {
        // Assign digital IO channels.
        this.clock = clock;
        this.data = data;

        // Prepare both digital channels for output.
        this.clock.setMode(DigitalChannel.Mode.OUTPUT);
        this.data.setMode(DigitalChannel.Mode.OUTPUT);

        // Prepare memory for the array of pixels.
        this.length = length;
        this.pixels = new Pixel[length];
        this.colors = new int[length * 3];

        for (int i = 0; i < length; i++) {
            pixels[i] = new Pixel(0, 0, 0);
        }
    }


    //----------------------------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------------------------

    /**
     * Mark all pixels to be reset during the next update.
     */
    public void clear() {
        for (Pixel pixel : pixels) {
            pixel.reset();
        }
    }


    /**
     * Mark all pixels to be reset, and optionally update them immediately.
     *
     * @param flush Whether to immediately update the pixels.
     */
    public void clear(boolean flush) {
        for (Pixel pixel : pixels) {
            pixel.reset();
        }

        if (flush) {
            update();
        }
    }


    /**
     * Get the number of pixels configured for use in the LED strip.
     *
     * @return Number of pixels configured for use in the LED strip.
     */
    public int getLength() {
        return length;
    }


    /**
     * Get the maximum output current (in amps) configured for use with the LED strip.
     *
     * This is important with respect to safety. Using DotStar LEDs at high intensity can draw a
     * large amount of current. To avoid damaging the digital IO controller, it is important to
     * set upper limits on the amount of current that can be drawn.
     *
     * By default, the value is 0.2 amps; this is the theoretical maximum of the Modern Robotics
     * Core Device Interface Module, and the lowest maximum currently known.
     *
     * @see #setMaxOutputAmps(double)
     */
    public double getMaxOutputAmps() {
        return maxOutputAmps;
    }


    /**
     * Set the number of pixels in the LED strip.
     *
     * @param length Number of pixels in the LED strip.
     */
    public void setLength(int length) {
        // Sanity check the LED length.
        if (length <= 0) {
            throw new IllegalArgumentException("LED strips must have at least one pixel");
        }

        this.length = length;

        // Create a new pixel array of the appropriate size.
        Pixel[] old_pixels = this.pixels;
        Pixel[] new_pixels = new Pixel[length];

        // Create a new colors array of the appropriate size.
        int[] old_colors = this.colors;
        int[] new_colors = new int[length * 3];

        // We will copy old array contents into the new arrays, as much as possible.
        int i;

        for (i = 0; i < Math.min(new_pixels.length, old_pixels.length); i++) {
            new_pixels[i] = old_pixels[i];

            new_colors[i * 3] = old_colors[i * 3];
            new_colors[i * 3 + 1] = old_colors[i * 3 + 1];
            new_colors[i * 3 + 2] = old_colors[i * 3 + 2];
        }

        // Initialize any remaining pixels as "off". The colors array elements are zero by default.
        for (; i < new_pixels.length; i++) {
            new_pixels[i] = new Pixel(0, 0, 0);
        }

        // Replace the old arrays.
        this.pixels = new_pixels;
        this.colors = new_colors;
    }


    /**
     * Set the maximum output current (in amps) allowed by the controller driving the LEDs.
     *
     * This is important with respect to safety. Using DotStar LEDs at high intensity can draw a
     * large amount of current. To avoid damaging the digital IO controller, it is important to
     * set upper limits on the amount of current that can be drawn.
     *
     * By default, the value is 0.2 amps; this is the theoretical maximum of the Modern Robotics
     * Core Device Interface Module, and the lowest maximum currently known.
     *
     * @param maxOutputAmps Theoretical maximum amperage allowed by the digital controller.
     * @see #getMaxOutputAmps()
     */
    public void setMaxOutputAmps(double maxOutputAmps) {
        // Sanity check amperage on the low side.
        if (maxOutputAmps < Pixel.ampsDrawn) {
            throw new IllegalArgumentException("Insufficient amperage for one LED pixel");
        }

        // Sanity check amperage on the high side.
        if (maxOutputAmps > 20) {
            throw new IllegalArgumentException("Theoretical output amperage is too high");
        }

        this.maxOutputAmps = maxOutputAmps;
    }


    /**
     * Flush the current array of pixels to the LED strip.
     *
     * Prior to writing colors to the pixel array, this method will calculate an estimation of the
     * total current (in amps) required to run the LEDs at the specified intensities. If this value
     * exceeds the theoretical maximum of the output device, then total intensities will be reduced
     * uniformly until the estimated current draw is below the the maximum.
     *
     * @see #setMaxOutputAmps(double)
     */
    public void update() {
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
            current += pixels[i].current();

            // When written out to the strip, we'll need the colors in BGR order.
            colors[i * 3] = pixels[i].blue;
            colors[i * 3 + 1] = pixels[i].green;
            colors[i * 3 + 2] = pixels[i].red;
        }

        // Ensure the total current will not exceed our theoretical maximum.
        if (current > maxOutputAmps) {
            double scale = current / maxOutputAmps;

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


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /**
     * Writes out the given buffer to the LEDs.
     *
     * This method will attempt to write the given data efficiently by only changing the clock line
     * if the current data bit is the same as the previous data bit.
     *
     * @param buffer Raw data to write out, including frame boundaries and termination bytes.
     */
    private synchronized void write(byte[] buffer) {
        // Track the previous data line state (to avoid repeatedly setting the same value).
        boolean new_state, old_state = false;

        // Clear the slate to start.
        data.setState(false);
        clock.setState(false);

        // Work our way through the buffer.
        for (byte bits : buffer) {
            for (int j = 0; j < 8; j++) {
                new_state = (bits & (0x01 << j)) > 0;

                // Don't waste time setting the data line if it should stay the same.
                if (new_state == old_state) {
                    clock.setState(true);
                    clock.setState(false);
                }
                else {
                    old_state = new_state;
                    data.setState(new_state);

                    clock.setState(true);
                    clock.setState(false);
                }
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
        public int red;

        /** Value of the blue channel, from 0 to 255. */
        public int blue;

        /** Value of the green channel, from 0 to 255. */
        public int green;

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
}
