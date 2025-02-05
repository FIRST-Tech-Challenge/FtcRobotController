package org.firstinspires.ftc.teamcode.StateMachine;

import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SampleScore;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SpecimenDelivery;
import org.firstinspires.ftc.teamcode.commands.placeCommands.SpecimenPlace;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

import java.util.Map;

public class Placing extends SelectCommand {

    public Placing(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                Map.of(
                        Constants.States.SAMPLE, new SampleScore(extension, pivot, chassis, gripper, controller),
                        Constants.States.SPECIMEN_DELIVERY, new SpecimenDelivery(extension, pivot, chassis, gripper, controller),
                        Constants.States.SPECIMEN_SCORE, new SpecimenPlace(extension, pivot, chassis, gripper, controller)
                ),
                RobotState.getInstance()::getState
        );
    }
}
