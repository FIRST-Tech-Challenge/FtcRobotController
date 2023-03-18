package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// TODO: 21/01/2023 Test
/**
 * Team 2's lift which lifts to a certain height
 */
public class Team2LiftComponent {
    /**
     * The Core Hex Motor used to move the lift from the bottom
     */
    public Motor motor;
    /**
     * The length of the bar the motor is attached to (m)
     */
    double barLength;
    /**
     * Encoder counts per radian rotation of motor
     */
    int countsPerRadian;
    /**
     * Initial height of the hand above ground (when bar is horizontal) (m)
     */
    double initialHeightOffset;

    /**
     * Create a {@code LiftComponent}
     * @param motor The Core Hex Motor used to move the lift from the bottom
     * @param barLength The length of the bar the motor is attached to (m)
     * @param countsPerRadian Encoder counts per radian of motor
     * @param initialHeightOffset Initial height of the hand above ground (when bar is horizontal) (m)
     */
    public Team2LiftComponent(Motor motor, double barLength, int countsPerRadian, double initialHeightOffset) {
        motor.setRunMode(Motor.RunMode.PositionControl); // So setTargetPosition works
        motor.setPositionCoefficient(0.05);

        this.motor = motor;
        this.barLength = barLength;
        this.countsPerRadian = countsPerRadian;
        this.initialHeightOffset = initialHeightOffset;
    }

    /**
     * Set the height of the hand
     * @param height The height of the hand relative to the floor (m)
     */
    public void setHeight(double height) throws InterruptedException {
        height -= initialHeightOffset; // Get height relative to starting position
        double angle = Math.asin(height/(2*this.barLength)); // arcsin(Opposite Length/Hypot Length )

        this.motor.setTargetPosition(motor.getCurrentPosition() + (int)(countsPerRadian*angle));

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
            this.motor.set(0.75);
            Thread.sleep(10);
        }

        this.motor.stopMotor(); // stop the motor
    }
}
