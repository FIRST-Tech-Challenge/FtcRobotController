package org.firstinspires.ftc.teamcode;

/**
 * Divides the LEDs into two separately colored segments.  We used it to
 * indicate two seperate states.
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

public class DSPatternHalfAndHalf extends DotStarPattern {

	public DSPatternHalfAndHalf(DotStarBridgedLED leds) {
    	super(leds);
    	isStatic = true;
    	numColors = 2;
    	colors.add(0, 0xC0C0C0);
    	colors.add(1, 0xFFD700);
    }

	public void update( ) {
	    if(colors.size() >= numColors) {
            int halfSize = leds.pixels.length / 2;
            // Update each pixel in the strip.
            for (int i = 0; i < halfSize; i++) {
                // Update individual pixels with their new color.
                leds.setPixel(i, colors.get(0));
            }
            for (int i = halfSize; i < leds.pixels.length; i++) {
                // Update individual pixels with their new color.
                leds.setPixel(i, colors.get(1));
            }

            // Flush the current set of colors to the strip.
            leds.update();
        }
	}
}