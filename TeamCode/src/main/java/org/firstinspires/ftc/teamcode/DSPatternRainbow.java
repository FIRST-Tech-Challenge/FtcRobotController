package org.firstinspires.ftc.teamcode;

/**
 * Creates a rainbow pattern through the pixels.
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

public class DSPatternRainbow extends DotStarPattern implements IDotStarPattern {
    /* This example will use a timer to create waves of color. */
    ElapsedTime timer;
    private double lastUpdateTime;

    public DSPatternRainbow(DotStarBridgedLED leds) {
    	super(leds);
        // Set up the timer we'll use for visual effects.
        patternDelay = 0.0;
        timer = new ElapsedTime();
        lastUpdateTime = timer.milliseconds();
    }

    public void update( ) {
        double currentTime = timer.milliseconds();
        if(currentTime - lastUpdateTime > patternDelay) {
            // Update each pixel in the strip.
            for (int i = 0; i < leds.pixels.length; i++) {

                // Calculate the new color based on its position and the current time.
                double hue = (i * 18 + timer.seconds() * 72) % 360;
                int color = Color.HSVToColor(new float[]{(float) hue, 1.0f, 0.25f});

                // Update individual pixels with their new color.
                leds.setPixel(i, color);
            }

            // Flush the current set of colors to the strip.
            leds.update();
        }
	}
}