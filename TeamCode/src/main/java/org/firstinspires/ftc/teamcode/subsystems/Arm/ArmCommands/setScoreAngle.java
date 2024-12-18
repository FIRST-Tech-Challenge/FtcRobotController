package org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;

import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;

public class setArmAngle extends BTCommand{

    PivotSubsystem m_pivot
    public setArmAngle(PivotSubsystem subsystem){
        m_pivot = subsystem;
    }

    public void execute(){
        m_pivot.setAngle()//tbd
    }

}