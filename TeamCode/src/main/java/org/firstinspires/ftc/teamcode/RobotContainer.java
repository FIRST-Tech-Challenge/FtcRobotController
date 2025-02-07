package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.*;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.StateMachine.Intaking;
import org.firstinspires.ftc.teamcode.StateMachine.Placing;
import org.firstinspires.ftc.teamcode.StateMachine.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public GripperSubsystem m_gripper;
    ExtensionSubsystem m_extension;
    PivotSubsystem m_pivot;
    ChassisSubsystem m_chassis;
    BTController m_controller;
    BTController m_controller2;
    Intaking intakeCommand;
    Placing placeCommand;
    Trigger Intake;
    Trigger Score;
    Trigger SpecimenDelivery;
    Trigger SpecimenScore;
    Trigger Sample;
    public RobotState state = RobotState.getInstance();

        public RobotContainer(HardwareMap map, BTController gamepad1, BTController gamepad2){

        m_controller = gamepad1;
        m_controller2 = gamepad2;

        Sample = new Trigger(m_controller.m_buttonsSuppliers[DPAD_UP.ordinal()]);
        SpecimenScore = new Trigger(m_controller.m_buttonsSuppliers[DPAD_RIGHT.ordinal()]);
        SpecimenDelivery = new Trigger(m_controller.m_buttonsSuppliers[DPAD_LEFT.ordinal()]);
        Score = new Trigger(m_controller.m_buttonsSuppliers[BUMPER_LEFT.ordinal()]);
        Intake = new Trigger(m_controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]);
        m_extension = new ExtensionSubsystem(map);
        m_gripper = new GripperSubsystem(map);
        m_chassis = new ChassisSubsystem(map);
        m_pivot = new PivotSubsystem(map, m_extension::getArmLength,m_chassis::getAcc);
        m_gripper.servoClaw.setPosition(0);
        m_gripper.isOpen = false;
        m_gripper.rotServo2.setPosition(score);
        resetGyro();
        configureBinds();

    }
    private void configureBinds() {

        Sample.whenActive(()->state.setState(Constants.States.SAMPLE));
        SpecimenDelivery.whenActive(()->state.setState(Constants.States.SPECIMEN_DELIVERY));
        SpecimenScore.whenActive(()->state.setState(Constants.States.SPECIMEN_SCORE));

        Intake.whenActive(new ConditionalCommand(intakeCommand,new InstantCommand(),placeCommand::isFinished),false);
        Score.whenActive(new ConditionalCommand(placeCommand,new InstantCommand(),intakeCommand::isFinished),false);

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                        () -> squareInput(-m_controller.getAxisValue(BTController.Axes.LEFT_Y_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.LEFT_X_axis)),
                        () -> squareInput(m_controller.getAxisValue(BTController.Axes.RIGHT_TRIGGER_axis) - m_controller.getAxisValue(BTController.Axes.LEFT_TRIGGER_axis))),
                true, LEFT_Y, LEFT_X, RIGHT_TRIGGER,LEFT_TRIGGER).whenInactive(m_chassis.stopMotor());

    }



    private Command resetGyro() {
        return new InstantCommand(()->m_chassis.gyro.reset());
    }

    public double squareInput(double input){
        return Math.signum(input)*Math.pow(input,2);
    }

    public void period(){
        dashboardTelemetry.addData("intakeIsFinished: ",intakeCommand::isFinished);
    }
}
