package com.hfrobots.tnt.corelib.drive.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

/*
 * Constants shared between multiple drive types.
 */
@Config
public abstract class DriveConstants {

    /*
     * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
     * fields may also be edited through the dashboard (connect to the robot's WiFi network and
     * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
     * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
     */
    //private static final MotorConfigurationType MOTOR_CONFIG =
    //        MotorConfigurationType.getMotorType(NeveRest20Orbital.class);
    private static final double TICKS_PER_REV = 537.6; //MOTOR_CONFIG.getTicksPerRev();

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience.
     */
    protected double wheelRadius = 2;
    protected double gearRatio = 1; // output (wheel) speed / input (motor) speed
    protected double trackWidth = 18;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */

    protected double kA = 0;
    protected double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public abstract DriveConstraints getBaseConstraints();

    public double encoderTicksToInches(int ticks) {
        return wheelRadius * 2 * Math.PI * gearRatio * ticks / TICKS_PER_REV;
    }

    public double rpmToVelocity(double rpm) {
        return rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0;
    }

    public double getMaxRpm() {
        return 315.0; /* return MOTOR_CONFIG.getMaxRPM() ; */
    }

    public double getKv() {
        return 1.0 / rpmToVelocity(getMaxRpm());
    }

    public double getKa() {
        return kA;
    }

    public double getKstatic() {
        return kStatic;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public abstract PIDCoefficients getTranslationalPID();

    public abstract PIDCoefficients getHeadingPid();
}
