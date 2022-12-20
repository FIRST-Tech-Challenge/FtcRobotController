package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;

public class DriveTele extends CommandBase {
    DriveSystem system;
    GamepadEx controls;
    
    public DriveTele(DriveSystem driveSystem, Gamepad driverPad) {
        system = driveSystem;
        controls = new GamepadEx(driverPad);
        
        addRequirements(driveSystem);
    }

    @Override
    public void execute() {
        // TODO: Strafing should smoothly change as the button is pressed
        int strafeInput = 0;
        
        if (controls.getButton(GamepadKeys.Button.LEFT_BUMPER))
            strafeInput--;
        if (controls.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            strafeInput++;
        
        system.setStrafe(strafeInput * Config.DRIVE_STRAFE_SENSITIVITY);
        system.setForward(curve(deadZone(controls.getLeftY())) * Config.DRIVE_FORWARD_SENSITIVITY);
        system.setTurn(curve(deadZone(controls.getRightX())) * Config.DRIVE_TURN_SENSITIVITY);
    }
    
    private double deadZone(double input) {
        if (Math.abs(input) < Config.DRIVE_DEAD_ZONE)
            return 0;
        
        return input;
    }
    
    private double curve(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), Config.DRIVE_CURVE);
    }
}
