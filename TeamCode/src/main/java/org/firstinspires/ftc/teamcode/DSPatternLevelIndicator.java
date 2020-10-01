package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

/**
 * A three color level indicator.  The range is set by maxValue and minValue.
 * The first color is shown at the minValue, the third color is shown at the
 * maxValue.  The second color is shown midway through the middle range.  The
 * colors shift from first to second to third through the ranges.
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

public class DSPatternLevelIndicator extends DotStarPattern {
    public double maxValue = 1.0;
    public double minValue = 0.0;
    public double maxThreshold = 0.67;
    public double minThreshold = 0.33;

	public DSPatternLevelIndicator(DotStarBridgedLED leds) {
    	super(leds);
    	isStatic = true;
    	numColors = 3;
    	colors.add(0, Color.RED);
    	colors.add(1, Color.YELLOW);
        colors.add(2, Color.GREEN);
    }

	public void update( ) {
	    if(colors.size() >= numColors) {
	        int lowColor = colors.get(0);
	        int midColor = colors.get(1);
	        int highColor = colors.get(2);
            int midLowColor = interpolateColors(lowColor, midColor, 0.5f);
            int midHighColor = interpolateColors(midColor, highColor, 0.5f);
	        int displayColor;

	        if(measuredValue < minThreshold) {
	            if(measuredValue < minValue) {
                    measuredValue = minValue;
                }
                // Go from color 0 to mid low color
                double percentage = (measuredValue - minValue) / (minThreshold - minValue);
                displayColor = interpolateColors(lowColor, midLowColor, percentage);
            } else if(measuredValue < maxThreshold) {
	            double midPoint = (maxThreshold  + minThreshold) / 2;
	            if(measuredValue < midPoint) {
                    double percentage = (measuredValue - minThreshold)/(midPoint - minThreshold);
                    displayColor = interpolateColors(midLowColor, midColor, percentage);
                } else {
                    double percentage = (measuredValue - midPoint)/(maxThreshold - midPoint);
                    displayColor = interpolateColors(midColor, midHighColor, percentage);
                }
            } else {
                if (measuredValue > maxValue) {
                    measuredValue = maxValue;
                }
                double percentage = (measuredValue - maxThreshold)/(maxValue - maxThreshold);
                displayColor = interpolateColors(midHighColor, highColor, percentage);
            }

            // Update each pixel in the strip.
            for (int i = 0; i < leds.pixels.length; i++) {
                // Update individual pixels with their new color.
                leds.setPixel(i, displayColor);
            }

            // Flush the current set of colors to the strip.
            leds.update();
        }
	}
}