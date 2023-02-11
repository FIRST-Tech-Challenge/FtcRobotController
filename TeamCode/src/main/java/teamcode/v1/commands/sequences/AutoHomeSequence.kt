package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.subsystems.Arm
import teamcode.v1.subsystems.Claw
import teamcode.v1.subsystems.Lift
import teamcode.v1.commands.subsystems.GuideCmds
import teamcode.v1.constants.GuideConstants
import teamcode.v1.subsystems.Guide

class AutoHomeSequence(
    lift: Lift,
    claw : Claw,
    arm : Arm,
    guide : Guide,
    firstArmAngle : Double,
    secondArmAngle : Double,
    liftHeight : Double,
    GripPos : Double
) : ParallelGroup(
    InstantCmd({arm.setPos(secondArmAngle)}, arm),
    ClawCmds.ClawOpenCmd(claw, guide, GuideConstants.telePos),
    InstantCmd({guide.setPos(GripPos)}),
    InstantCmd({lift.setPos(liftHeight)}),
)

