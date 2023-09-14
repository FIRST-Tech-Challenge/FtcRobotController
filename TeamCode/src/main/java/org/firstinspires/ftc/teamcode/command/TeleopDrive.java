package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drivePose.DrivePose;

public class TeleopDrive  extends CommandBase {

    private final DrivePose mDrive;
    private final Gamepad mGamepad;
    private final  double driveKp = 0.3;
    public TeleopDrive (DrivePose drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
//        mDrive.mecanumFieldDrive(mGamepad.left_stick_x * driveKp
//                , mGamepad.left_stick_y * driveKp
//                , mGamepad.right_stick_x * driveKp, false);
//        mDrive.mecanumCentricDrive(mGamepad.left_stick_x * driveKp
//                                 , mGamepad.left_stick_y * driveKp
//                                 , mGamepad.right_stick_x * driveKp, false);
        mDrive.driveJoy(-mGamepad.left_stick_y, mGamepad.left_stick_x, mGamepad.right_stick_x);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}