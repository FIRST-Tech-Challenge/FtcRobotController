package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.extensionPIDCosntants.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperCommands.CloseGripper;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;
    ChassisSubsystem m_chassis;

    Gamepad gamepad1;
    Gamepad gamepad2;
    BTController m_controller;


    public RobotContainer(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2){
        m_extension = new ExtensionSubsystem(map);
        m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        m_controller = new BTController(gamepad1);
        resetGyro();
        configureBinds();
    }
    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }
    private void configureBinds() {
        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.left_y.getAsDouble()),
                        () -> squareInput(m_controller.left_x.getAsDouble()),
                        () -> squareInput(m_controller.right_trigger.getAsDouble() - m_controller.left_trigger.getAsDouble())),
                true, LEFT_Y, LEFT_X, RIGHT_TRIGGER,LEFT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(m_pivot.set(),false,DPAD_RIGHT);
        m_controller.assignCommand(m_pivot.disablePID(),true,DPAD_LEFT);
        m_controller.assignCommand(m_pivot.enablePID(),true,DPAD_UP);
        m_controller.assignCommand(setPickup(), false,BUTTON_RIGHT);
        m_controller.assignCommand(m_extension.disablePID(), false,BUTTON_LEFT);
        m_controller.assignCommand(m_extension.enablePID(), false,BUTTON_UP);
//        m_controller.assignCommand(m_extension.setNegative(), false,BUTTON_DOWN);
//        m_controller.assignCommand(m_gripper.CloseGripper(),false,BUTTON_RIGHT);
//        m_controller.assignCommand(m_gripper.OpenGripper(),false,BUTTON_LEFT);
//        m_controller.assignCommand(m_gripper.setPickup(),false,BUTTON_DOWN);
    }

    private void resetGyro() {
        m_chassis.gyro.reset();
    }

    public Command setPickup(){
        return new ParallelCommandGroup(m_extension.setExtension(),m_pivot.set());
    }

}
