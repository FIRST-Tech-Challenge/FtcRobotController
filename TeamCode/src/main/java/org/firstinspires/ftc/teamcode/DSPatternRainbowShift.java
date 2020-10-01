package org.firstinspires.ftc.teamcode;

/**
 * Based on the rainbow pattern, but instead of having the rainbow run through
 * the pixels, all the pixels shift through the rainbow.
 *
 * DotStar LEDs (i.e. https://www.adafruit.com/product/2238) are collections of LEDs which are
 * programmable using SPI. While it is possible to use two digital outputs as data and clock lines,
 * an I2C/SPI Bridge can manage the digital writes at a much higher frequency. This is required for
 * "smooth" color changes. However, a similar class for driving the LEDs via two digital outputs
 * is also available.
 *
 * @author AJ Foster and Rick Van Smith
 * @version 2.0.0
 */
import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DSPatternRainbowShift extends DotStarPattern implements IDotStarPattern {
    /* This example will use a timer to create waves of color. */
    ElapsedTime timer;
    private double lastUpdateTime;
    private int hueCount;

    public DSPatternRainbowShift(DotStarBridgedLED leds) {
    	super(leds);
        // Set up the timer we'll use for visual effects.
        patternDelay = 0.0;
        timer = new ElapsedTime();
        lastUpdateTime = timer.milliseconds();
        hueCount = 0;
    }

    public void update( ) {
        double currentTime = timer.milliseconds();
        if(currentTime - lastUpdateTime > patternDelay) {
            double hue = (hueCount * 18 + timer.seconds() * 72) % 360;
            hueCount++;
            if(hueCount >= leds.pixels.length) {
                hueCount = 0;
            }
            // Update each pixel in the strip.
            for (int i = 0; i < leds.pixels.length; i++) {
                int color = Color.HSVToColor(new float[]{(float) hue, 1.0f, 0.25f});

                // Update individual pixels with their new color.
                leds.setPixel(i, color);
            }

            // Flush the current set of colors to the strip.
            leds.update();
        }
	}
}