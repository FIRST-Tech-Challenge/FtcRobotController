package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_UP;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.DPAD_UP;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.LEFT_TRIGGER;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.LEFT_X;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.LEFT_Y;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.RIGHT_X;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisCommands.*;
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


    public RobotContainer(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2){
        m_extension = new ExtensionSubsystem(map);
        //m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength, m_extension::getArmCOM);

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
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());

    }

    private void resetGyro() {

    }

}
