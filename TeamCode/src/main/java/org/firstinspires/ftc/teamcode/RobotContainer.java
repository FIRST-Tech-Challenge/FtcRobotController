package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;

    public RobotContainer(HardwareMap map){
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength,m_extension::getArmCOM);
    }

}
