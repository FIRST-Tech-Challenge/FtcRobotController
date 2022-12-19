package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;

public class DriveTele extends CommandBase {
    DriveSystem system;
    GamepadEx driveOp;
    
    public DriveTele(DriveSystem driveSystem, ElevatorSystem elevatorSystem, GamepadEx driverControls) {
        system = driveSystem;
        driveOp = driverControls;
        
        addRequirements(driveSystem);
        //addRequirements(elevatorSystem);
    }

    @Override
    public void execute() {
        int strafeInput = 0;
        
        if (driveOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
            strafeInput--;
        if (driveOp.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            strafeInput++;
        
        //system.setElevator(driveOp.getRightY());
    }
}
