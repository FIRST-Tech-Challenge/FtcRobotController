package org.firstinspires.ftc.teamcode;

/**
 * Two colors, one background color and one chase color.  The chase color runs through the
 * background color.  Changing chaseSeparation will change the number of pixels between chasers.
 * Changing chaseSpeed will change how many milliseconds for the chase pixel to move to the next
 * pixel.
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

public class DSPatternChase extends DotStarPattern implements IDotStarPattern {
    /* This pattern will run a chase pixel through a solid color.  Changing
     * chaseSpeed will change the time in milliseconds to shift the pixels
     * and chaseSeperation will change how far the chase goes until the next
     * chase begins. */
    private ElapsedTime timer;
    private double lastUpdateTime;
    private int chasePosition;

    public DSPatternChase(DotStarBridgedLED leds) {
    	super(leds);
        // Set up the timer we'll use for visual effects.
        timer = new ElapsedTime();
        lastUpdateTime = timer.milliseconds();
        numColors = 2;
        pixelSpacing = 10;
        patternDelay = 0.0;
        colors.add(0, Color.RED);
        colors.add(1, Color.BLUE);
        chasePosition = 0;
        isStatic = false;
    }

    public void update( ) {
        if(colors.size() >= numColors) {
            // Wait for time to pass
            double currentTime = timer.milliseconds();
            if(currentTime - lastUpdateTime >= patternDelay) {
                chasePosition++;
                lastUpdateTime = currentTime;
                if (chasePosition == pixelSpacing) {
                    chasePosition = 0;
                }
                int midColor = interpolateColors(colors.get(0), colors.get(1), 0.5);
                int currentPosition = chasePosition;

                // Update each pixel in the strip.
                for (int i = 0; i < leds.pixels.length; i++) {
                    if(i == currentPosition) {
                        leds.setPixel(i, colors.get(1));
                        currentPosition += pixelSpacing;
                    } else if(i == currentPosition - 1) {
                        leds.setPixel(i, midColor);
                    } else {
                        leds.setPixel(i, colors.get(0));
                    }
                }
                // Flush the current set of colors to the strip.
                leds.update();
            }
        }
	}
}