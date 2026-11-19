package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Two positional servos geared to one output (the turret ring). They must never be told to turn in
 * different directions or to different positions, or they fight through the gears and slip teeth.
 *
 * <p>The pair is configured once with ONE direction and ONE PWM range, verified to match, and every
 * position write goes to both servos together. There is deliberately no way to command one alone.
 *
 * <p>Software can only make the commands identical. Positional servos pick their own speed with their
 * internal controller, so equal speed also needs equal Axon programmer settings on both servos
 * (PWM Power, Soft Start, travel, neutral, direction); see doc/SoftwareConfiguration.md.
 */
public final class PairedServos {
    private final Servo first, second;
    private final String description;
    private double position = Double.NaN;

    public PairedServos(String description, Servo first, Servo second, Servo.Direction direction,
                        int minPulseUs, int maxPulseUs) {
        this.description = description;
        if (first == null || second == null) throw new IllegalArgumentException(description + ": both servos are required");
        if (first == second) throw new IllegalArgumentException(description + ": both names resolve to the same servo");
        if (!(first instanceof PwmControl) || !(second instanceof PwmControl)) {
            throw new IllegalArgumentException(description + ": both servos must support PwmControl so their PWM ranges match");
        }
        this.first = first;
        this.second = second;
        PwmControl.PwmRange range = new PwmControl.PwmRange(minPulseUs, maxPulseUs);
        for (Servo servo : new Servo[]{first, second}) {
            servo.setDirection(direction);
            ((PwmControl) servo).setPwmRange(range);
        }
        verifyMatched();
    }

    /** Throws if the two servos would turn a shared position command into different shaft angles. */
    public void verifyMatched() {
        if (first.getDirection() != second.getDirection()) {
            throw new IllegalStateException(description + ": servo directions differ (" + first.getDirection()
                    + " vs " + second.getDirection() + "); the servos would turn opposite ways");
        }
        PwmControl.PwmRange a = ((PwmControl) first).getPwmRange(), b = ((PwmControl) second).getPwmRange();
        if (a == null || b == null || a.usPulseLower != b.usPulseLower || a.usPulseUpper != b.usPulseUpper) {
            throw new IllegalStateException(description + ": servo PWM ranges differ; one position would be two different angles");
        }
    }

    /** The only write path: both servos receive the same position in the same loop, back to back. */
    public void setPosition(double position) {
        if (!Double.isFinite(position)) return;
        this.position = position;
        first.setPosition(position);
        second.setPosition(position);
    }

    /** Last position sent to both servos; NaN before the first write. */
    public double getPosition() { return position; }
}
