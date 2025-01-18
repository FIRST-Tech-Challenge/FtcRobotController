package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.extensionPIDCosntants.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
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


    public RobotContainer(HardwareMap map, BTController gamepad1){
        m_extension = new ExtensionSubsystem(map);
        //m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength);

        m_controller = gamepad1;
        resetGyro();
        configureBinds();
    }
    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }
    private void configureBinds() {
        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.getAxisValue(BTController.Axes.LEFT_Y_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.LEFT_X_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.RIGHT_TRIGGER_axis) - m_controller.getAxisValue(BTController.Axes.LEFT_TRIGGER_axis))),
                true, LEFT_Y, LEFT_X, RIGHT_TRIGGER,LEFT_TRIGGER).whenInactive(m_chassis.stopMotor());
//        m_controller.assignCommand(setScore(), false,BUTTON_RIGHT);
//        m_controller.assignCommand(setIdle(), false,BUTTON_UP);
//        m_controller.assignCommand(setPickup(), false,BUTTON_LEFT);
//        m_controller.assignCommand(m_extension.setNegative(), false,BUTTON_DOWN);
//        m_controller.assignCommand(m_gripper.CloseGripper(),false,BUTTON_RIGHT);
//        m_controller.assignCommand(m_gripper.OpenGripper(),false,BUTTON_LEFT);
//        m_controller.assignCommand(m_gripper.setPickup(),false,BUTTON_DOWN);
    }

    private void resetGyro() {
        m_chassis.gyro.reset();
    }

    public Command setScore(){
        return new ParallelCommandGroup(m_extension.setExtension(extended),m_pivot.set(score));
    }
    public Command setIdle(){
        return new ParallelCommandGroup(m_extension.setExtension(closed),m_pivot.set(idle));
    }
    public Command setPickup(){
        return new SequentialCommandGroup(m_pivot.set(closed), m_extension.setExtension(extended));
    }

    public void period(){
        //can be used for general telemetry
    }

}
