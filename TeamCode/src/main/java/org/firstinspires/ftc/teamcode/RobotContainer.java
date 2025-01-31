package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    public GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;
    ChassisSubsystem m_chassis;
    BTController m_controller;


    public RobotContainer(HardwareMap map, BTController gamepad1){
        m_extension = new ExtensionSubsystem(map);
        m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength);
        m_gripper.servoClaw.setPosition(0);
        m_gripper.isOpen = false;
        m_gripper.rotServo2.setPosition(score);
        m_controller = gamepad1;
        resetGyro();
        configureBinds();
    }
    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }
    private void configureBinds() {
        initStateMachine();
        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.getAxisValue(BTController.Axes.LEFT_Y_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.LEFT_X_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.RIGHT_TRIGGER_axis) - m_controller.getAxisValue(BTController.Axes.LEFT_TRIGGER_axis))),
                true, LEFT_Y, LEFT_X, RIGHT_TRIGGER,LEFT_TRIGGER).whenInactive(m_chassis.stopMotor());
        m_controller.assignCommand(togglePickup(),false,BUMPER_RIGHT);
//        m_controller.assignCommand(setScore(), false,BUTTON_RIGHT);
//        m_controller.assignCommand(setIdle(), false,BUTTON_UP);
//        m_controller.assignCommand(setIntake(), false,BUTTON_LEFT);
//        m_controller.assignCommand(togglePickup(), false,BUTTON_DOWN);
//        m_controller.assignCommand(setScoreLow(),false,BUMPER_LEFT);
    }


    private void resetGyro() {
        m_chassis.gyro.reset();
    }

    public Command initStateMachine(){
        return new RepeatCommand(
                new SequentialCommandGroup(
                setIdle(),
                    new WaitUntilCommand(m_controller.getTrigger(BUTTON_DOWN)::get),
                setIntake(),
                    new WaitUntilCommand(m_controller.getTrigger(BUTTON_DOWN)::get),
                setIdle(),
                    new WaitUntilCommand(m_controller.getTrigger(BUTTON_DOWN)::get),
                setScore(),
                    new WaitUntilCommand(m_controller.getTrigger(BUTTON_DOWN)::get),
                m_gripper.openClaw(),
                    new WaitUntilCommand(m_controller.getTrigger(BUTTON_DOWN)::get)
            )
        );
    }

    public Command setScore() {
        return m_chassis.slowDriving(0.4).andThen(m_pivot.set(scoreMidpoint))
                .andThen(new WaitUntilCommand(()->m_pivot.m_pivotPID.atGoal())).withTimeout(1200)
                .andThen(new ParallelCommandGroup(
                m_extension.setExtension(extended),
                m_pivot.set(score), m_gripper.setScore())
        );
    }
        public Command setScoreLow(){
        return new ParallelCommandGroup(
                m_extension.setExtension(extended),
                m_pivot.set(half),m_gripper.setScore()
        );
    }
    public Command setIdle(){
        return m_pivot.set(idle)
                .andThen(m_gripper.setScore())
                .andThen(new WaitCommand(600))
                .andThen(m_extension.setExtension(closed)
                .andThen(m_chassis.stopSlowDriving())

                );
    }
    public Command setIntake(){
        return m_pivot.set(midpoint)
                .andThen(m_gripper.setPickup())
                .andThen(m_extension.setExtension(half))//should be half changed for autonomous
                .andThen(m_chassis.slowDriving(0.4));
    }

    private Command togglePickup() {
        return new RepeatCommand(
                new SequentialCommandGroup(
                m_pivot.set(pickup)
                    .andThen(new WaitUntilCommand(m_controller.getTrigger(BUMPER_RIGHT)::get)),
                m_gripper.closeClaw()
                    .andThen(new WaitUntilCommand(m_controller.getTrigger(BUMPER_RIGHT)::get)),
                m_pivot.set(up)
                    .andThen(new WaitUntilCommand(m_controller.getTrigger(BUMPER_RIGHT)::get))
                ).interruptOn(m_controller.getTrigger(BUTTON_DOWN)::get)
            );
    }

    public void period(){
        //can be used for general telemetry
    }

}
