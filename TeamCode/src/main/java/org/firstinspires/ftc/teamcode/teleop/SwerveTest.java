package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

@TeleOp(name="Swerve Test", group="Worlds")
public class SwerveTest extends ConfiguredOpMode {

    private final SwerveDriveBase swerve = new SwerveDriveBase(RobotConstants.enabledModules.LEFT);

    @Override
    public void superInit() {

    }

    @Override
    public void registerTriggers() {

    }

    @Override
    public void superInit_Loop() {

    }

    @Override
    public void superStart() {

    }

    @Override
    public void superLoop() {
        swerve.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, (1-gamepad1.right_trigger), true);
    }

    @Override
    public void superStop() {

    }
}
