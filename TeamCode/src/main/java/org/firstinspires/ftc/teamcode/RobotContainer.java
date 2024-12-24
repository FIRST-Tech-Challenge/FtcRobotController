package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;

    Gamepad gamepad1;
    Gamepad gamepad2;
    BTController m_controller;

    public RobotContainer(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2){
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength,m_extension::getArmCOM);
        m_extension = new ExtensionSubsystem(map);
        m_gripper = new GripperSubsystem(map);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        
    }

}
