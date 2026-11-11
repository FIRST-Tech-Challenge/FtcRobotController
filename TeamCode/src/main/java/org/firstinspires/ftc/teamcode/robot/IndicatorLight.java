package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * One goBILDA RGB Indicator Light (PWM controlled). It plugs into a servo port and is set up as a
 * servo in the robot configuration; the servo position picks the color.
 *
 * The light keeps its color until it is told otherwise, so it's fine to set it every loop.
 */
public class IndicatorLight {

    // Servo positions for each color. RED and GREEN were tested last season.
    public static double OFF = 0.0;
    public static double RED = 0.31;
    public static double YELLOW = 0.4;   // first guess -- check on the robot
    public static double GREEN = 0.5;

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
