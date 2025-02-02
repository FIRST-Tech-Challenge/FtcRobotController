package org.firstinspires.ftc.teamcode.commands.states;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.closed;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.half;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.idle;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.midpoint;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;

public class Idle extends SequentialCommandGroup{
    public Idle(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper){
        super(
                pivot.set(idle),
                gripper.setScore(),
                new WaitCommand(600),
                extension.setExtension(closed),
                chassis.stopSlowDriving()
        );
    }
}
