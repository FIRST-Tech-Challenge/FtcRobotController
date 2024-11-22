package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveWithGamepadCommand extends CommandBase {

    private Gamepad gamepad;
    private Drive drive;

    public DriveWithGamepadCommand(Gamepad gamepad, Drive drive) {
        this.gamepad = gamepad;
        this.drive = drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double forward = gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        forward = -forward;

        if (Math.abs(strafe) <= Math.abs(forward)){
            strafe = 0;
        }else {
            strafe = Math.copySign(Math.abs(strafe) - Math.abs(forward), strafe);
        }

        if (gamepad.left_bumper){
            forward *= 0.75;
            turn *= 0.75;
            strafe *= 0.75;
        }
        else {
            forward *= 1;
            turn *= 0.95;
            strafe *= 1;
        }

        drive.arcadeDrive(forward, turn, strafe, true);
    }

    @Override
    public boolean isFinished() { return false;}


    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}