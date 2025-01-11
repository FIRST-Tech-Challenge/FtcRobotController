package org.firstinspires.ftc.teamcode.subsystems.Arm.ArmCommands;

import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;

public class setScoreAngle extends BTCommand{

    PivotSubsystem m_pivot;
    public setScoreAngle(PivotSubsystem subsystem){
        m_pivot = subsystem;
    }

    public void execute(){
//        m_pivot.setAngle(0);//tbd
    }


}