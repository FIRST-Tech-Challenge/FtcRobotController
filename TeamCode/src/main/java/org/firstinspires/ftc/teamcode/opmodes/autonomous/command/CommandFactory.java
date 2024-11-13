package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

public class CommandFactory {
    private AutoMecanumDriveTrain driveTrain;
    private RollingIntake intake;
    private LimeLight vision;
    private Telemetry telemetry;
    private DeliveryPivot pivot;
    private DeliverySlider slider;

    public CommandFactory(Telemetry telemetry, AutoMecanumDriveTrain driveTrain, RollingIntake intake, LimeLight vision, DeliveryPivot pivot, DeliverySlider slider) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY);
    }

    public TurnAngleCommand turnAngle(double angleInDegrees) {
        return new TurnAngleCommand(driveTrain, telemetry, angleInDegrees);
    }

    public TurnAngleRelativeCommand turnAngleRelative(double angleInDegrees) {
        return new TurnAngleRelativeCommand(driveTrain, telemetry, angleInDegrees);
    }
}
