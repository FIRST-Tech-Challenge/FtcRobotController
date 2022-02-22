package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.SuppressLint;

/**
 * The class that all {@link MovementException} and {@link org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning} extend from
 */
public class MovementException extends Exception {

    /**
     * A constructor that allows creation without incurring the runtime cost of a stack trace
     * Requires Android Version 24
     */
    @SuppressLint("NewApi")
    public MovementException() {
        super("", null, true, false);
    }

    /**
     * When called from {@link org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.NavigationalDrivetrain} the arguments of moveToPosition or moveTowardsPosition are passed as well
     *
     * @param x                The X coordinate moving towards
     * @param y                The Y coordinate moving towards
     * @param theta            The angle to move towards
     * @param tolerance        The tolerance for the movement
     * @param telemetry        A telemetry object for logging
     * @param gps              To determine location
     * @param _isStopRequested To return if stop is requested
     * @param _opModeIsActive  To return if opMode is no longer active
     * @param voltageSensor    A voltage sensor to get voltage
     * @throws MovementException Throws as the subclass desires
     */
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
    }

}
