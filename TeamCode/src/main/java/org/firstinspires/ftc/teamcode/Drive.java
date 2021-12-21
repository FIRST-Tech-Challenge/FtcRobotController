package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

@TeleOp(name = "Drive")
public class Drive extends CommandBasedTeleOp
{
    DriveTrainSubsystem driveTrain;
    TankDriveCommand tankDriveCommand;
    ArcadeDriveCommand arcadeDriveCommand;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        tankDriveCommand = new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y);
        arcadeDriveCommand = new ArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_x, () -> -gamepad1.left_stick_y, () -> gamepad1.right_stick_x);

        driveTrain.setDefaultCommand(tankDriveCommand);

        gp1.x().whileHeld(arcadeDriveCommand);
    }
}