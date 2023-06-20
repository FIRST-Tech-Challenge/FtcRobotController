package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;

/**
 * Lift with pulleys which lifts to a certain height
 */
public class LiftComponent {
    /**
     * The Core Hex Motor used to move the lift from the bottom
     */
    public Motor motor;
    /**
     * Counts of the motor per m
     */
    int countsPerMeter;
    /**
     * Initial height of the hand above ground (m)
     */
    double initialHeightOffset;

    private final boolean showTelemetry = true;

    /**
     * Create a {@code LiftComponent}
     * @param motor The Core Hex Motor used to move the lift from the bottom
     * @param barLength The length of the bar the motor is attached to (m)
     * @param countsPerMeter Encoder counts per radian of motor
     * @param initialHeightOffset Initial height of the hand above ground (when bar is horizontal) (m)
     */
    public LiftComponent(Motor motor, double barLength, int countsPerMeter, double initialHeightOffset) {
        motor.setRunMode(Motor.RunMode.PositionControl); // So setTargetPosition works

        this.motor = motor;
        this.countsPerMeter = countsPerMeter;
        this.initialHeightOffset = initialHeightOffset;
    }

    /**
     * Set the height of the hand
     * @param height The height of the hand relative to the floor (m)
     */
    public void setHeight(double height) throws InterruptedException {
        height -= initialHeightOffset; // Get height relative to starting position
        Double angle = null; // arcsin(Opposite Length/Hypot Length )

        this.motor.setTargetPosition(motor.getCurrentPosition() + (int)(countsPerMeter *angle));

        // set the tolerance
        this.motor.setPositionTolerance(13.6);   // allowed maximum error

        while (!this.motor.atTargetPosition()) {
            this.motor.set(0.75);
            Thread.sleep(10);
        }

        this.motor.stopMotor(); // stop the motor
    }

    public void setPosition(int counts) throws InterruptedException {
        this.motor.setTargetPosition(motor.getCurrentPosition() + counts);

        // set the tolerance
        this.motor.setPositionTolerance(13.6);   // allowed maximum error

        while (!this.motor.atTargetPosition()) {
            this.motor.set(0.25);
            Thread.sleep(10);
        }

        this.motor.stopMotor(); // stop the motor
    }
}
