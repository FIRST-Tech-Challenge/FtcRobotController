package org.firstinspires.ftc.teamcode.subsystems.Gripper.gripperCommands;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.gripperSubsystem;

public class closeGripper extends BTCommand {
    public gripperSubsystem m_subsystem;

    public closeGripper(gripperSubsystem subsystem){
        m_subsystem = subsystem;
    }

    public void execute(){
        m_subsystem.isOpen = false;
    }
}
