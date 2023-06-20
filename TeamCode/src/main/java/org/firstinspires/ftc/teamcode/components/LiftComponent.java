package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;

/**
 * Lift with pulleys which lifts to a certain height
 */
public class LiftComponent {
    /**
     * An enum that contains the possible lift positions
     */
    public enum LiftPosition {
        GROUND(0),
        MIDDLE(300),
        HIGH(1500);

        private final int counts;
        LiftPosition(int counts) {
            this.counts = counts;
        }

        public double getCounts(){
            return this.counts;
        }
    }
    /**
     * The Core Hex Motor used to move the lift from the bottom
     */
    public Motor motor;


    /**
     * The number of counts needed to reach the initial position
     */
    private final int initialPositionCounts;

    /**
     * Create a {@code LiftComponent}
     * @param motor The Motor used to move the lift
     */
    public LiftComponent(Motor motor, LiftPosition initialPosition) {
        // Reset the encoder counts to start from the initial position
        motor.resetEncoder();
        initialPositionCounts = initialPosition.counts;

        this.motor = motor;
        setTargetPosition(initialPosition);
    }

    /**
     * Start moving the motor to a given position
     * @param position The desired final position of the arm
     */
    public void setTargetPosition(LiftPosition position) {
        this.motor.setRunMode(Motor.RunMode.PositionControl); // So setTargetPosition works
        this.motor.setPositionTolerance(15);
        this.motor.setPositionCoefficient(0.01);

        int countsFromInitialPosition = position.counts - initialPositionCounts;

        this.motor.setTargetPosition(countsFromInitialPosition);
    }

    /**
     * Runs the motor at the given speed in an appropriate mode. Needs to be called every tick
     */
    public void set(double speed){
        this.motor.set(speed);
    }


    /**
     * Sets motor run mode
     */
    public void setRunMode(Motor.RunMode runMode) {
        motor.setRunMode(runMode); // So setTargetPosition works
    }
}
