package org.firstinspires.ftc.teamcode.koawalib.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class ArmCmds {
    open class ArmCmd(arm: Arm, pos: Double) : InstantCmd({ arm.setPos(pos) }, arm)

    class ArmHomeCmd(arm: Arm) : ArmCmd(arm, Arm.homePos)
    class ArmGroundCmd(arm: Arm) : ArmCmd(arm, Arm.groundPos)
    class ArmLowCmd(arm: Arm) : ArmCmd(arm, Arm.lowPos)
    class ArmMidCmd(arm: Arm) : ArmCmd(arm, Arm.midPos)
    class ArmHighCmd(arm: Arm) : ArmCmd(arm, Arm.highPos)
}