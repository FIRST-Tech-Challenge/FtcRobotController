package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

public class TeleopDrive  extends CommandBase {

    private final DrivePose mDrive;
//    private final MyCamera myCamera;
    private final Gamepad mGamepad;
    private ElapsedTime timer = new ElapsedTime();
    private final  double driveKp = 0.3;
    public TeleopDrive (DrivePose drive, Gamepad gamepad) {
        mDrive = drive;
        mGamepad = gamepad;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
//        mDrive.mecanumFieldDrive(mGamepad.left_stick_x * driveKp
//                , mGamepad.left_stick_y * driveKp
//                , mGamepad.right_stick_x * driveKp, false);
//        mDrive.mecanumCentricDrive(mGamepad.left_stick_x * driveKp
//                                 , mGamepad.left_stick_y * driveKp
//                                 , mGamepad.right_stick_x * driveKp, false);

        mDrive.driveJoy(-mGamepad.left_stick_x, -mGamepad.left_stick_y, mGamepad.right_stick_x);
//        mDrive.driveJoy(-mGamepad.left_stick_y, mGamepad.left_stick_x, mGamepad.right_stick_x);//normal drive

//        mDrive.driveField(mGamepad.left_stick_y, mGamepad.left_stick_x, mGamepad.right_stick_x);

//        mDrive.autoMoveXY(timer.milliseconds());

//        double[] id = myCamera.getAprilTagIDData(10);
//        mDrive.driveAlign(id[0], id[1], id[2], id[3]);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}