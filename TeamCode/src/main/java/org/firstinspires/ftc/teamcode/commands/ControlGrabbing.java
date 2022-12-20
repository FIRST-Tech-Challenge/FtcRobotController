package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;

public class ControlGrabbing extends CommandBase {
    ElevatorSystem system;
    GamepadEx controls;
    
    public ControlGrabbing(ElevatorSystem system, Gamepad systemsPad) {
        this.system = system;
        this.controls = new GamepadEx(systemsPad);
        
        addRequirements(system);
    }

    @Override
    public void execute() {
        system.setVelocity(controls.getLeftY() * Config.ELEVATOR_SPEED);
    }
}
