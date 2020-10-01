package org.firstinspires.ftc.teamcode;

/**
 * Represents a pattern to be executed by the pattern library.  This abstract
 * class implements implements most of the mundane functions of the pattern
 * interface.  A new pattern should extend this class and implement the
 * IDotStarPattern interface.
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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.DotStarBridgedLED.Pixel;

public abstract class DotStarPattern implements IDotStarPattern {
    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    protected DotStarBridgedLED leds;
	protected List<Integer> colors;
	protected boolean isStatic;
	protected int numColors;
	protected int pixelSpacing;
	protected double patternDelay;
	protected double measuredValue;

	public DotStarPattern(DotStarBridgedLED leds) {
	    isStatic = false;
	    colors = Collections.synchronizedList(new ArrayList<Integer>());
	    this.leds = leds;
	    numColors = 0;
	    pixelSpacing = 0;
	    patternDelay = 0.0;
	    measuredValue = 0.0;
	}

	/**
	 * Gets the pixels used by the pattern.
	 * @return The list of pixels used by the pattern.
	 */
	public Pixel[] getLeds() {return leds.pixels;}

	/**
	 * Gets the number of colors the chosen pattern uses.
	 * @return The number of colors that should be passed into setPatternColors.
	 */
	public int getNumColors() { return numColors; }

    /**
     * Sets an external value used by the pattern to adjust the pattern.
     * @param measurement A value used to adjust the pattern.  The pattern must be
     *   resent to adjust to the new value.
     */
    public void setMeasuredValue(double measurement) { measuredValue = measurement; }

    /**
     * Set colors of a pattern.  This default implementation should work for
	 * most  patterns.
     * @param newColors Color value (android.graphics.Color)
     */
	public void setPatternColors(List<Integer> newColors) {
	    if(newColors != null) {
            colors = Collections.synchronizedList(new ArrayList<Integer>(newColors));
        }
	}

    /**
     * Get colors of a pattern.  This default implementation should work for
	 * most  patterns.
     * @return The list of colors that will make up the pattern.
     */
    public List<Integer> getPatternColors() {
        return new ArrayList<Integer>(colors);
    }

    /**
     * Set static display of a pattern.  This default implementation should work for
	 * most  patterns.
     * @param isStatic boolean true if the pattern does not change, false if
     *  the pattern changes over time.
     */
	public void setStaticPattern(boolean isStatic) {
		this.isStatic = isStatic;
	}

    /**
     * Get if this is a static display pattern.
	 * @return boolean true if the pattern is set to be static, false if it is
	 *   a moving pattern.
     */
	public boolean isStaticPattern() {
		return this.isStatic;
	}

	/**
	 * The pattern decides how to use the pattern delay.  Typically this would
	 * be the time between shifting the pattern.
	 * @param milliseconds A time in milliseconds to delay the pattern.
	 */
	public void setPatternDelay(double milliseconds) { patternDelay = milliseconds; };

	/**
	 * Gets the time the pattern will use as a delay.
	 * @return The time in milliseconds the pattern will delay, exact usage is
	 * defined by the pattern.
	 */
	public double getPatternDelay() { return patternDelay; };

	/**
	 * The pattern decides how to use the pattern spacing.  Typically this would
	 * be how many pixels between pattern elements.
	 * @param pixelSpacing A spacing in pixels.
	 */
	public void setPatternSpacing(int pixelSpacing) { this.pixelSpacing = pixelSpacing; };

	/**
	 * Gets the spacing the pattern will use.
	 * @return The spacing in pixels the pattern will use, exact usage is
	 * defined by the pattern.
	 */
	public int getPatternSpacing() { return pixelSpacing; };

	/**
	 * Gives the color between two colors.  The percentage is how far towards
	 * color2 to go.  This is a utility function that many color patterns might
	 * use.
	 * @param color1 int The first color to interpolate
	 * @param color2 int The second color to interpolate
	 * @param percentage double How far towards the color2 to go.  100% would
	 *                   be fully color2.
	 * @return int The new color.
	 */
	protected int interpolateColors(int color1, int color2, double percentage) {
		int alpha1 = Color.alpha(color1);
		int red1 = Color.red(color1);
		int green1 = Color.green(color1);
		int blue1 = Color.blue(color1);
		int alpha2 = Color.alpha(color2);
		int red2 = Color.red(color2);
		int green2 = Color.green(color2);
		int blue2 = Color.blue(color2);
		int newAlpha = (int)(alpha1 * (1 - percentage) + alpha2 * percentage);
		int newRed = (int)(red1 * (1 - percentage) + red2 * percentage);
		int newGreen = (int)(green1 * (1 - percentage) + green2 * percentage);
		int newBlue = (int)(blue1 * (1 - percentage) + blue2 * percentage);

		return Color.argb(newAlpha, newRed, newGreen, newBlue);
	}
}