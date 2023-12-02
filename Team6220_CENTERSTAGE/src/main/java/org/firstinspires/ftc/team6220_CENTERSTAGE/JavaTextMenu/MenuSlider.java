package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A type of menu element for choosing a value inside a range.
 */
public class MenuSlider implements HoverableMenuElement<Double> {

	private boolean isHovered;
	private double value, min, max, scale;

	/**
	 * creates a new slider.
	 * @param min the minimum value allowed
	 * @param max the maximum value allowed
	 * @param scale scales the units per character when displayed
	 * @param defaultValue the starting value, instead of using minimum
	 */
	public MenuSlider(double min, double max, double scale, double defaultValue) {
		this.min = min;
		this.max = max;
		this.scale = scale;
		this.value = defaultValue;
	}
	/**
	 * creates a new slider.
	 * @param min the minimum value allowed
	 * @param max the maximum value allowed
	 * @param scale scales the units per character when displayed
	 */
	public MenuSlider(double min, double max, double scale) {
		this(min, max, scale, min);
	}
	/**
	 * creates a new slider.
	 * @param min the minimum value allowed
	 * @param max the maximum value allowed
	 */
	public MenuSlider(double min, double max) {
		this(min, max, 1.0, min);
	}

	// MenuElement interface required methods
	
	public String getAsString() {
		String asString = isHovered ? ">[" : " [";
		double sliderLength = (this.max - this.min) * this.scale;
		for (int i = 0; i < Math.round(sliderLength); i++) {
			asString += i < (this.value - this.min) * this.scale ? "/" : "-";
		}
		return asString + "] " + (Math.round(this.value * 10.0) / 10.0);
	}

	// HoverableMenuElement interface required methods

	public void showHover(boolean showHover) {
		this.isHovered = showHover;
	}

	public void updateWithInput(MenuInput input) {
		double nextValue = this.value + input.getX() / this.scale;
		this.value = clamp(nextValue, this.min, this.max);
	}

	public boolean isCompleted() {
		return true;
	}

	public Double result() {
		return this.value;
	}

	// clamps value between a minimum and maximum value
	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
