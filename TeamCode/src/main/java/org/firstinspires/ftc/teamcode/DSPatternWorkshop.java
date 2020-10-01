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

public class DSPatternWorkshop extends DotStarPattern {

    boolean blueEvens = false;
	public DSPatternWorkshop(DotStarBridgedLED leds) {
    	super(leds);
    	isStatic = false;
    	numColors = 2;
    	colors.add(0, Color.RED);
    	colors.add(1, Color.BLUE);
    }

	public void update( ) {
	    if(colors.size() >= numColors) {
            // Update each pixel in the strip.
            for (int i = 0; i < leds.pixels.length; i++) {
                if(blueEvens) {
                    if (i % 2 == 0) {
                        leds.setPixel(i, colors.get(1));
                    } else {
                        leds.setPixel(i, colors.get(0));
                    }
                } else {
                    if (i % 2 == 0) {
                        leds.setPixel(i, colors.get(0));
                    } else {
                        leds.setPixel(i, colors.get(1));
                    }
                }
            }
            // Update individual pixels with their new color.
            blueEvens = !blueEvens;

            // Flush the current set of colors to the strip.
            leds.update();
        }
	}
}