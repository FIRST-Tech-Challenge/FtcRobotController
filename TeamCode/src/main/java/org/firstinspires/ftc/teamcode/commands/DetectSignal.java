package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.systems.SignalSystem;

public class DetectSignal extends CommandBase {
    private SignalSystem system;
    
    public DetectSignal(SignalSystem system) {
        this.system = system;
        
        addRequirements(system);
    }

    @Override
    public void initialize() {
        system.startCamera();
    }

    @Override
    public void end(boolean interrupted) {
        system.stopCamera();
    }
}
