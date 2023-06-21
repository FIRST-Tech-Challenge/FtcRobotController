package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

public class TrapezoidalProfileMotor {
    public static class TrapezoidalProfile {
        private final double maxSpeedSquared;
        private final double maxAccelerationMagnitude;

        /** Settings for a trapezoidal profile
         * @param maxSpeed the maximal speed the motor can have (counts/s)
         * @param maxAccelerationMagnitude maximal magnitude of acceleration (counts/s^2)
         */
        public TrapezoidalProfile(double maxSpeed, double maxAccelerationMagnitude) {
            this.maxSpeedSquared = maxSpeed * maxSpeed;
            this.maxAccelerationMagnitude = maxAccelerationMagnitude;
        }
    }
    public Motor motor;

    public TrapezoidalProfile profile;
    private int startCounts;
    private int endCounts;

    /** A motor that can move to a point using a trapezoidal profile
     * @param motor the motor to use
     * @param profile the settings to use
     */
    public TrapezoidalProfileMotor(Motor motor, TrapezoidalProfile profile) {
        this.motor = motor;
        this.profile = profile;
        this.motor.setVeloCoefficients(0.05, 0.01, 0.31);
        this.motor.setFeedforwardCoefficients(0.92, 0.47);
    }

    /**
     * Sets the target position to the target counts and starts moving towards it.
     * Requires the run method to be called periodically
     */
    public void setTargetPosition(int counts){
        this.startCounts = this.motor.getCurrentPosition();
        this.endCounts = counts;
        this.motor.setRunMode(Motor.RunMode.VelocityControl);
    }

    /**
     * Calculates the squared speed using SUVAT
     * NOTE: the speed always has a positive sign
     * @return the desired speed at that distance from target
     */
    private double calculateVSquared(double distance){
        // v^2 - u^2 = 2 * a * s
        return 2 * profile.maxAccelerationMagnitude * Math.abs(distance);
    }

    /**
     * Do the calculations and run the motor towards the target
     */
    public void run(){
        int counts = this.motor.getCurrentPosition();
        int distanceToTarget = this.endCounts - counts;
        int distanceFromStart = counts - startCounts;
        // Velocity prediction based on distance from start
        // Corresponds to accelerating from start
        double v1 = calculateVSquared(distanceFromStart);
        // Same, but velocity from end corresponding to a deceleration
        double v2 = calculateVSquared(distanceToTarget);
        double v = Math.min(Math.min(v1, v2), profile.maxSpeedSquared);
        // Remember, this is squared speed without a sign
        // Give it a sign and remove the square
        v = Math.signum(distanceToTarget) * Math.sqrt(v);

        // Run at that velocity
        // Note the division - this is so that we convert our ticks per second into a fraction
        // As the method expects it
        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("Lift desired velocity", v);
        t.addData("Lift set input", v / this.motor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
        this.motor.set(v / this.motor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }
}
