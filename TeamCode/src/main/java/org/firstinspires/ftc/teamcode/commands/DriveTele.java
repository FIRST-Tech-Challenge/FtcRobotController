package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

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
        int strafeInput = 0;
        
        if (driveOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
            strafeInput--;
        if (driveOp.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            strafeInput++;
        
        system.setStrafe(strafeInput);
        system.setForward(driveOp.getLeftY());
        system.setTurn(driveOp.getRightX());
    }
}
