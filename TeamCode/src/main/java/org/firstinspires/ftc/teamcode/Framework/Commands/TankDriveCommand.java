package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Framework.subsystems.TankDriveTrain;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankDriveTrain drive;
    private DoubleSupplier right;
    private DoubleSupplier left;

    public TankDriveCommand(TankDriveTrain drive, DoubleSupplier left, DoubleSupplier right){
        this.drive = drive;
        this.right = right;
        this.left = left;

        addRequirements(drive);
    }

    @Override
    public void initialize(){
        this.drive.setPower(0, 0);
    }

    @Override
    public void execute(){
        this.drive.setPower(left.getAsDouble(), right.getAsDouble());
    }


}
