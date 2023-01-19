package teamcode.v1.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.LoopCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.constants.ArmConstants
import teamcode.v1.subsystems.Arm

class ArmCmds {
    open class ArmCmd(arm: Arm, pos: Double) : InstantCmd({ arm.setPos(pos) }, arm)

    class ArmHomeCmd(arm: Arm) : ArmCmd(arm, ArmConstants.homePos)
    class ArmGroundCmd(arm: Arm) : ArmCmd(arm, ArmConstants.groundPos)
    class ArmLowCmd(arm: Arm) : ArmCmd(arm, ArmConstants.lowPos)
    class ArmMidCmd(arm: Arm) : ArmCmd(arm, ArmConstants.midPos)
    class ArmHighCmd(arm: Arm) : ArmCmd(arm, ArmConstants.highPos)
}