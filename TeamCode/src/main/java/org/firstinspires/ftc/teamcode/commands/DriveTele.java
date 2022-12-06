package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.systems.DriveSystem;

public class DriveTele extends CommandBase {
    DriveSystem system;
    GamepadEx driveOp;
    
    public DriveTele(DriveSystem driveSystem, GamepadEx driverControls) {
        system = driveSystem;
        driveOp = driverControls;
        
        addRequirements(driveSystem);
    }

    @Override
    public void execute() {
        system.drive(driveOp.getLeftX(), driveOp.getLeftY(), driveOp.getRightX());
    }
}
