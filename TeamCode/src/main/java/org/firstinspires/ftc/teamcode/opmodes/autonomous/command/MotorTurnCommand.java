package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import com.google.common.util.concurrent.Uninterruptibles;

import java.util.concurrent.TimeUnit;

public class MotorTurnCommand extends SounderBotCommandBase {

    AutoMecanumDriveTrain driveTrain;

    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    double targetX, targetY, targetHeading;

    boolean isFirst = true;


    public MotorTurnCommand(AutoMecanumDriveTrain driveTrain, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
    }

    @Override
    public void doExecute() {
        this.driveTrain.setWheelsPower(.5, 0, 0, 0);
        Uninterruptibles.sleepUninterruptibly(1000, TimeUnit.MILLISECONDS);

        this.driveTrain.setWheelsPower(0, .5, 0, 0);
        Uninterruptibles.sleepUninterruptibly(1000, TimeUnit.MILLISECONDS);

        this.driveTrain.setWheelsPower(0, 0, 0.5, 0);
        Uninterruptibles.sleepUninterruptibly(1000, TimeUnit.MILLISECONDS);

        this.driveTrain.setWheelsPower(0, 0, 0, 0.5);
        Uninterruptibles.sleepUninterruptibly(1000, TimeUnit.MILLISECONDS);

        this.driveTrain.setWheelsPower(0, 0, 0, 0);
        Uninterruptibles.sleepUninterruptibly(1000, TimeUnit.MILLISECONDS);


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
