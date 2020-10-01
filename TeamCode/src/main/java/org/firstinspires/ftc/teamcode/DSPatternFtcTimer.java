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

public class DSPatternFtcTimer extends DotStarPattern implements IDotStarPattern {
    ElapsedTime timer;
    private double targetTime = 0.0;
    private boolean flashOn = false;

    public DSPatternFtcTimer(DotStarBridgedLED leds) {
    	super(leds);
        // Set up the timer we'll use for visual effects.
        timer = new ElapsedTime();
        numColors = 4;
        targetTime = 0.0;
        patternDelay = 120000.0;
        colors.add(0, Color.GREEN);
        colors.add(1, Color.YELLOW);
        colors.add(2, Color.RED);
        colors.add(3, Color.BLACK);
        isStatic = false;
    }

    public void update( ) {
        if (colors.size() >= numColors) {
            if(targetTime == 0.0) {
                targetTime = timer.milliseconds() + patternDelay;
            }

            int color;
            double timeRemaining = targetTime - timer.milliseconds();
            // This is relatively hard coded for now.
            // For the last 10 seconds flash red
            if(timeRemaining < 10000.0) {
                if (flashOn) {
                    flashOn = false;
                    color = colors.get(2);
                } else {
                    flashOn = true;
                    color = colors.get(3);
                }
            } else if(timeRemaining < 20000.0) {
                color = interpolateColors(colors.get(2), colors.get(1), (timeRemaining - 10000.0)/(10000.0));
            } else {
                color = interpolateColors(colors.get(1), colors.get(0), (timeRemaining - 20000.0)/(100000.0));
            }
            for (int i = 0; i < leds.pixels.length; i++) {
                // Update individual pixels with their new color.
                leds.setPixel(i, color);
            }

            // Flush the current set of colors to the strip.
            leds.update();
        }
    }
}