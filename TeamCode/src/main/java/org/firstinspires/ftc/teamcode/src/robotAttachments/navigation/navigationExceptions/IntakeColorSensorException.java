package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;

/**
 * Tests to see if the intake is not empty
 */
public class IntakeColorSensorException extends MovementException {

    /**
     * Internal intake to monitor
     */
    private final Outtake outtake;

    /**
     * A constructor
     *
     * @param outtake A outtake objects to read from
     */
    public IntakeColorSensorException(Outtake outtake) {
        super();
        this.outtake = outtake;
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (outtake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
            throw this;
        }
    }
}
