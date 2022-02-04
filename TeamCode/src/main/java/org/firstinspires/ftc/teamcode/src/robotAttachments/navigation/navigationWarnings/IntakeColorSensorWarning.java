package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;

public class IntakeColorSensorWarning extends MovementWarning {

    private final ContinuousIntake intake;

    public IntakeColorSensorWarning(ContinuousIntake intake) {
        super();
        this.intake = intake;
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
            throw this;
        }
    }
}
