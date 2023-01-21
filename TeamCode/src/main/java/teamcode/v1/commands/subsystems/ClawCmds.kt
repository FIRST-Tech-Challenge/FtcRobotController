package teamcode.v1.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.constants.ClawConstants
import teamcode.v1.subsystems.Claw
import teamcode.v1.subsystems.Guide

class ClawCmds {
    open class ClawCmd(claw: Claw, pos: Double) : InstantCmd({ claw.setPos(pos) }, claw)

    class ClawCloseCmd(claw: Claw) : ClawCmd(claw, ClawConstants.closePos)
    class ClawOpenCmd(claw: Claw, guide : Guide, GripPos: Double) : SequentialGroup(
        ClawCmd(claw, ClawConstants.openPos),
        InstantCmd({guide.setPos(GripPos)})
    )
}