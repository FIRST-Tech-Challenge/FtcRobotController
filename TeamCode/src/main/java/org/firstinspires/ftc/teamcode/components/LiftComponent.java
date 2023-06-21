package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.TrapezoidalProfileMotor;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * Lift with pulleys which lifts to a certain height
 */
@Config
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
     * Static to allow it being read from the dashboard
     */
    public Motor motor;


    public TrapezoidalProfileMotor trapezoidalProfileMotor;

    /**
     * The number of counts needed to reach the initial position
     */
    private final int initialPositionCounts;

    /**
     * Create a {@code LiftComponent}
     * @param motor The Motor used to move the lift
     */
    public LiftComponent(Motor motor, LiftPosition initialPosition) {
        trapezoidalProfileMotor = new TrapezoidalProfileMotor(motor, new TrapezoidalProfileMotor.TrapezoidalProfile(30, 30));
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
        trapezoidalProfileMotor.setTargetPosition(position.counts);
    }

    /**
     * Runs the motor at the given speed in an appropriate mode. Needs to be called every tick
     */
    public void set(double speed){
        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("Lift motor speed", motor.getCorrectedVelocity());
        trapezoidalProfileMotor.run();
    }
}
