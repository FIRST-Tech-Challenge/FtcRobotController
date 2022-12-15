package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.systems.OdometrySystem;

public class UpdatePose extends CommandBase {
    OdometrySystem system;
    
    public UpdatePose(OdometrySystem system) {
        this.system = system;
        
        addRequirements(system);
    }
}
