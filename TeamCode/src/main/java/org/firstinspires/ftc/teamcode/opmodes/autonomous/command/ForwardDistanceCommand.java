package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;

public class ForwardDistanceCommand extends SounderBotCommandBase {

    public ForwardDistanceCommand(AutoMecanumDriveTrain driveTrain, double expectedDistance, long timeout, Telemetry telemetry) {
        super(timeout);

        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.expectedDistance = expectedDistance;
    }

    double expectedDistance;

    AutoMecanumDriveTrain driveTrain;
    Telemetry telemetry;

    @Override
    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        double distance = this.driveTrain.GetForwardDistanceFromObstacleInMM();

        telemetry.addData("Forward distance", distance);
        telemetry.update();

        return Math.abs(distance - expectedDistance) < 10;
    }

    boolean finished = false;

    @Override
    public void doExecute() {
    }
}
