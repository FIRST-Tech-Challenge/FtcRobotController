package teamcode.mule.commands

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.mule.constants.MuleArmConstants
import teamcode.mule.subsystems.MuleArm

class MuleArmCommands(arm: MuleArm, firstPos: Double, secondPos: Double) : SequentialGroup(
    InstantCmd({arm.setPos(firstPos)}, arm),
    WaitCmd(0.8),
    InstantCmd({arm.setPos((secondPos))}, arm)
)