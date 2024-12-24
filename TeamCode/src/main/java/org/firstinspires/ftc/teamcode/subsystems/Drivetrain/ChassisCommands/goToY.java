package org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;

public class goToY extends BTCommand {
    ChassisSubsystem m_chassis;

    public goToY(Subsystem subsystem){
        m_chassis = subsystem;
    }

    public void execute(){}
}

