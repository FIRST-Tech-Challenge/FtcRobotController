package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;
public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    private final exampleSubsytem m_exmapleSubsystem;
    public RobotContainer(HardwareMap map){
        m_exmapleSubsystem= new exampleSubsytem(map);

    }
    //bind commands to trigger
    public void bindCommands(){
    }

    public Command AutonomousCommand(){
        return null;
    }
}
