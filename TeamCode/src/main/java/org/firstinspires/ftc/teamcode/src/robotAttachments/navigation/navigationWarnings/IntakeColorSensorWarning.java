package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.src.utills.Executable;

public class IntakeColorSensorWarning extends MovementWarning {

    private final Outtake intake;

    public IntakeColorSensorWarning(Outtake intake) {
        super();
        this.intake = intake;
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        //if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
        //    throw this;
        //}
    }
}
