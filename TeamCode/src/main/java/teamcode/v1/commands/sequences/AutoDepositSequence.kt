package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.subsystems.*

class AutoDepositSequence(
        lift: Lift,
        arm : Arm,
        claw: Claw,
        guide : Guide,
        whacker: Whacker,
        armAngle : Double,
        LiftHeight : Double,
        GripPos : Double,
        whackPos : Double,
) : SequentialGroup(
        ClawCmds.ClawCloseCmd(claw),
        InstantCmd({arm.setPos(armAngle)}, arm),
        WaitCmd(0.3),
        InstantCmd({whacker.setPos(whackPos)}, whacker),
        WaitCmd(0.3),
        InstantCmd({lift.setPos(LiftHeight)}, lift),
        WaitCmd(0.1),
        InstantCmd({guide.setPos(GripPos)}),
)