package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class AlignDriveTrainToSpecimenDelivery extends SounderBotCommandBase {

    public AlignDriveTrainToSpecimenDelivery(AutoMecanumDriveTrain driveTrain, double expectedDistance, long timeout, Telemetry telemetry) {
        super(timeout);

        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.expectedDistance = expectedDistance;
    }

    AutoMecanumDriveTrain driveTrain;
    Telemetry telemetry;

    double expectedDistance;

    @Override
    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        return finished;
    }

    boolean finished = false;

    @Override
    public void doExecute() {
        double distance = this.driveTrain.GetForwardDistanceFromObstacleInMM();
        telemetry.addData("Forward distance", distance);
        telemetry.update();

        if(Math.abs(distance - expectedDistance) < 15) {
            driveTrain.stop();
            finished = true;
        } else {
            double power = Math.signum(distance - expectedDistance) * .15;
            driveTrain.setWheelsPower(power, power, power, power);
        }
    }
}
