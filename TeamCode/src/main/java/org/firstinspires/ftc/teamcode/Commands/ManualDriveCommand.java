package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Common.MyRobot;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {

    private final MyRobot robot;
    private final DriveSubsystem driveSubsystem;

    public ManualDriveCommand(MyRobot robot) {
        this.robot = robot;
        driveSubsystem = robot.driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Gamepad gamepad = robot.gamepad1();
        driveSubsystem.arcadeDrive(-gamepad.left_stick_y, gamepad.right_stick_x);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
