package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class TeleopDrive  extends CommandBase {

    private final Drive mDrive;
    private final Gamepad mGamepad;
    private final  double driveKp = 0.5;
    public TeleopDrive (Drive drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mDrive.mecanumFieldDrive(mGamepad.left_stick_x * driveKp
                , mGamepad.left_stick_y * driveKp
                , mGamepad.right_stick_x * driveKp, false);
//        mDrive.mecanumCentricDrive(mGamepad.left_stick_x * driveKp
//                                 , mGamepad.left_stick_y * driveKp
//                                 , mGamepad.right_stick_x * driveKp, false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}