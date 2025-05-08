package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class AlignDriveTrainToSpecimenDelivery extends SounderBotCommandBase {

    public AlignDriveTrainToSpecimenDelivery(DriveTrain driveTrain, double expectedDistance, double absoluteMin, long timeout, Telemetry telemetry) {
        super(timeout);

        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.expectedDistance = expectedDistance;
        this.absoluteMin = absoluteMin;
    }

    DriveTrain driveTrain;
    Telemetry telemetry;

    double expectedDistance;

    double absoluteMin;

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

        double absError = Math.abs(distance - expectedDistance);

        if(!finished) {

            if (absError < 15) {
                driveTrain.stop();
                finished = true;
            } else if (absError < 50) {
                double power = Math.signum(distance - expectedDistance) * .13;
                driveTrain.setWheelsPower(power, power, power, power);
            } else {
                double power = Math.signum(distance - expectedDistance) * .23;
                driveTrain.setWheelsPower(power, power, power, power);
            }

            if (distance <= absoluteMin) {
                driveTrain.stop();
                finished = true;
            }
        }
    }
}
