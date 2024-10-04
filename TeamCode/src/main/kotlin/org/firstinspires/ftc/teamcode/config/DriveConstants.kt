package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
object DriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    const val TICKS_PER_REV: Double = 537.6
    const val MAX_RPM: Double = 312.0

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    const val RUN_USING_ENCODER: Boolean = false
    @JvmStatic
    var MOTOR_VELO_PID: PIDFCoefficients = PIDFCoefficients(
        0.0, 0.0, 0.0,
        getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)
    )

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    var WHEEL_RADIUS: Double = 1.8898 // in
    var GEAR_RATIO: Double = 1.0 // output (wheel) speed / input (motor) speed
    var TRACK_WIDTH: Double = 13.5 // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    @JvmStatic
    var kV: Double = (1.0 / rpmToVelocity(MAX_RPM)) * 1.05
    @JvmStatic
    var kA: Double = 0.003
    @JvmStatic
    var kStatic: Double = 0.01

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    var MAX_VEL: Double = 52.48291908330528
    var MAX_ACCEL: Double = 52.48291908330528
    var MAX_ANG_VEL: Double = Math.toRadians(247.6056)
    var MAX_ANG_ACCEL: Double = Math.toRadians(231.31152000000003)

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    fun rpmToVelocity(rpm: Double): Double {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0
    }

    fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }
}