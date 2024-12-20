package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class IntakeFromWall extends SounderBotCommandBase {

    public IntakeFromWall(AutoMecanumDriveTrain driveTrain, RollingIntake intake) {
        super(4000);

        this.intake = intake;
        this.driveTrain = driveTrain;
    }

    RollingIntake intake;

    AutoMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        intake.IntakeInAuto();
        sleep(200);
        driveTrain.setWheelsPower(.15, .15, .15, .15);
    }

    @Override
    protected boolean isTargetReached() {
        boolean finished = intake.IsSampleIntaken() || driveTrain.getOdo().getPosX() < 100;
        if(finished) {
            intake.HoldInAuto();
            driveTrain.stop();
        }

        return finished;
    }

    @Override
    public void doExecute() {
    }
}
