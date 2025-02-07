package org.firstinspires.ftc.teamcode.StateMachine;

import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.SampleIntake;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.SpecimenIntake;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.SpecimenPickup;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

import java.util.Map;

public class Intaking extends SelectCommand {

    public Intaking(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                Map.of(
                        Constants.States.SAMPLE, new SampleIntake(extension, pivot, chassis, gripper, controller),
                        Constants.States.SPECIMEN_DELIVERY, new SpecimenIntake(extension, pivot, chassis, gripper, controller),
                        Constants.States.SPECIMEN_SCORE, new SpecimenPickup(extension, pivot, chassis, gripper, controller)
                ),
                RobotState.getInstance()::getState
        );
        addRequirements(pivot);
    }

}
