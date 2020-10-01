package org.firstinspires.ftc.teamcode;

/**
 * Two colors, one background color and one twinkle color.  The twinkle color shows up randomly
 * through the background color.  Changing ledsPerTwinkler will change the number of twinkle
 * pixels on the strip.  Changing twinkleSpeed will change how many milliseconds for the twinkle
 * pixels to move.
 *
 * DotStar LEDs (i.e. https://www.adafruit.com/product/2238) are collections of LEDs which are
 * programmable using SPI. While it is possible to use two digital outputs as data and clock lines,
 * an I2C/SPI Bridge can manage the digital writes at a much higher frequency. This is required for
 * "smooth" color changes. However, a similar class for driving the LEDs via two digital outputs
 * is also available.
 *
 * @author Rick Van Smith
 * @version 1.0.0
 */
import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DSPatternTwinkle extends DotStarPattern implements IDotStarPattern {
    ElapsedTime timer;
    private double lastUpdateTime;

    public DSPatternTwinkle(DotStarBridgedLED leds) {
    	super(leds);
        // Set up the timer we'll use for visual effects.
        timer = new ElapsedTime();
        pixelSpacing = 10;
        patternDelay = 0.0;
        lastUpdateTime = timer.milliseconds();
        numColors = 2;
        colors.add(0, Color.BLUE);
        colors.add(1, Color.WHITE);
        isStatic = false;
    }

    public void update( ) {
        if (colors.size() >= numColors) {
            // Wait for time to pass
            double currentTime = timer.milliseconds();
            if(currentTime - lastUpdateTime > patternDelay) {
                int twinkleLeds = leds.pixels.length / pixelSpacing;
                // Update each pixel in the strip.
                for (int i = 0; i < leds.pixels.length; i++) {
                    // Update individual pixels with their new color.
                    leds.setPixel(i, colors.get(0));
                }
                // Set the twinklers
                for (int i = 0; i < twinkleLeds; i++) {
                    int twinklePos = (int) (Math.random() * leds.pixels.length);
                    leds.setPixel(twinklePos, colors.get(1));
                }

                // Flush the current set of colors to the strip.
                leds.update();
            }
        }
    }
}