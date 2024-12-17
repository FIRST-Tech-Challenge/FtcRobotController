package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.Arm.extensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.pivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.gripperSubsystem;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    gripperSubsystem m_gripper;
    extensionSubsystem m_extension;
    pivotSubsystem m_pivot;

    public RobotContainer(HardwareMap map){
        m_pivot = new pivotSubsystem(map, m_extension::getArmLength);
    }

}
