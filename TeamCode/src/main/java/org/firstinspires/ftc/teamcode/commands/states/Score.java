package org.firstinspires.ftc.teamcode.commands.states;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.extended;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.score;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.scoreMidpoint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;

public class Score extends SequentialCommandGroup{
    public Score(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper){
        super(
                chassis.slowDriving(0.4),
                pivot.set(scoreMidpoint),
                new WaitUntilCommand(()->pivot.m_pivotPID.atGoal()).withTimeout(1200),
                new ParallelCommandGroup(
                    extension.setExtension(extended),
                    pivot.set(score), gripper.setScore())
        );
    }
}
