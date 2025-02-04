package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.commands.intakeCommands.SampleIntake;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.SpecimenIntake;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.SpecimenPickup;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SampleScore;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SpecimenDelivery;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SpecimenPlace;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    public GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;
    ChassisSubsystem m_chassis;
    BTController m_controller;
    BTController m_controller2;
    Trigger Intake;
    Trigger Score;
    Trigger SpecimenDelivery;
    Trigger SpecimenScore;
    Trigger Sample;
    SampleScore sampleScoreCommand;
    SampleIntake sampleIntakeCommand;
    SpecimenIntake specimenIntakeCommand;
    SpecimenDelivery specimenDeliveryCommand;
    SpecimenPickup specimenPickupCommand;
    SpecimenPlace specimenPlaceCommand;

private States currentState = States.SAMPLE;
    public RobotContainer(HardwareMap map, BTController gamepad1, BTController gamepad2){
        m_extension = new ExtensionSubsystem(map);
        m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength,m_chassis::getAcc);
        m_gripper.servoClaw.setPosition(0);
        m_gripper.isOpen = false;
        m_gripper.rotServo2.setPosition(score);
        m_controller = gamepad1;
        m_controller2 = gamepad2;
        specimenIntakeCommand = new SpecimenIntake(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        specimenDeliveryCommand = new SpecimenDelivery(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        specimenPickupCommand = new SpecimenPickup(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        specimenPlaceCommand = new SpecimenPlace(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        sampleScoreCommand = new SampleScore(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        sampleIntakeCommand = new SampleIntake(m_extension,m_pivot,m_chassis,m_gripper,m_controller);
        Sample = new Trigger(m_controller.m_buttonsSuppliers[DPAD_UP.ordinal()]);
        SpecimenScore = new Trigger(m_controller.m_buttonsSuppliers[DPAD_RIGHT.ordinal()]);
        SpecimenDelivery = new Trigger(m_controller.m_buttonsSuppliers[DPAD_LEFT.ordinal()]);
        Score = new Trigger(m_controller.m_buttonsSuppliers[BUMPER_LEFT.ordinal()]);
        Intake = new Trigger(m_controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]);
        resetGyro();
        configureBinds();
    }
    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }
    private void configureBinds() {
//        m_controller.assignCommand(new ConditionalCommand(scoreCommand,new WaitUntilCommand(()->intakeCommand.isFinished()).andThen(scoreCommand),()->!intakeCommand.isScheduled()),false,BUMPER_LEFT);
//        m_controller.assignCommand(new ConditionalCommand(intakeCommand,new WaitUntilCommand(()->scoreCommand.isFinished()).andThen(intakeCommand),()->!scoreCommand.isScheduled()),false,BUMPER_RIGHT);
//        m_controller.assignCommand(new RepeatCommand(new StateMachine(m_extension,m_pivot,m_chassis,m_gripper,m_controller)),false,DPAD_UP);
        Score.whenActive(setPlace());
        Intake.whenActive(setIntake());
        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.getAxisValue(BTController.Axes.LEFT_Y_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.LEFT_X_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.RIGHT_TRIGGER_axis) - m_controller.getAxisValue(BTController.Axes.LEFT_TRIGGER_axis))),
                true, LEFT_Y, LEFT_X, RIGHT_TRIGGER,LEFT_TRIGGER).whenInactive(m_chassis.stopMotor());

    }

    private Command setIntake(){
        return new SelectCommand(
                Map.of(
                        States.SAMPLE, sampleIntakeCommand,
                        States.SPECIMEN_DELIVERY, specimenIntakeCommand,
                        States.SPECIMEN_SCORE, specimenIntakeCommand
                ),
                () -> currentState
        );
    }

    private Command setPlace(){
        return new SelectCommand(
                Map.of(
                        States.SAMPLE, sampleScoreCommand,
                        States.SPECIMEN_DELIVERY, specimenDeliveryCommand,
                        States.SPECIMEN_SCORE, specimenPlaceCommand
                ),
                () -> currentState
        );
    }



    private Command resetGyro() {
        return new InstantCommand(()->m_chassis.gyro.reset());
    }

    private static enum States{
        SPECIMEN_SCORE,
        SAMPLE,
        SPECIMEN_DELIVERY
    }
}
