package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;

public class TelemetryCommand extends SounderBotCommandBase {

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    double targetX, targetY, targetHeading;

    boolean isFirst = true;


    public TelemetryCommand(AutoMecanumDriveTrain driveTrain, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
    }

    @Override
    public void doExecute() {
        odo.update();
        GoBildaPinpointDriver.Pose2D pose = odo.getPosition();

        if(true) {

            double x = pose.getX();
            double y = pose.getY();
            double botHeading = Math.toDegrees(pose.getHeading());

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("h", botHeading);

            telemetry.update();
        }
    }

    @Override
    protected boolean isTargetReached() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}