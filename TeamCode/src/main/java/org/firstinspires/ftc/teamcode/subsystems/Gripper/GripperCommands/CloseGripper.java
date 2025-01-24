package org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperCommands;

import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;

public class CloseGripper extends BTCommand {
    public GripperSubsystem m_subsystem;

    public CloseGripper(GripperSubsystem subsystem){
        m_subsystem = subsystem;
    }

    public void execute(){

        m_subsystem.isOpen = false;
    }
}
