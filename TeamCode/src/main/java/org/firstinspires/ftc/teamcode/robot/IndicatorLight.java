package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * One goBILDA RGB Indicator Light (PWM controlled). It plugs into a servo port and is set up as a
 * servo in the robot configuration; the servo position picks the color.
 *
 * The light keeps its color until it is told otherwise, so it's fine to set it every loop.
 */
public class IndicatorLight {

    // Servo positions for each color (checked with the Light Test OpMode).
    // RED and BLUE are kept for showing the alliance; PINK is the "not ready" status color.
    public static double OFF = 0.0;
    public static double RED = 0.31;
    public static double YELLOW = 0.37;
    public static double GREEN = 0.5;
    public static double BLUE = 0.59;
    public static double PINK = 0.69;

    private final Servo servo;
    private double color = Double.NaN;   // last color sent; NaN = nothing sent yet

    public IndicatorLight(Servo servo) {
        this.servo = servo;
    }

    /** Show a color, e.g. IndicatorLight.GREEN. */
    public void setColor(double color) {
        if (color == this.color) return; // already showing it; skip the extra hardware write
        this.color = color;
        servo.setPosition(color);
    }

    public void off() {
        setColor(OFF);
    }
}
