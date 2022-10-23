package org.firstinspires.ftc.teamcode.koawalib.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class LiftCmds {
    open class LiftCmd(lift: Lift, pos: Double) : InstantCmd({ lift.setPos(pos) }, lift)

    class LiftHomeCmd(lift: Lift) : LiftCmd(lift, Lift.homePos)
    class LiftGroundCmd(lift: Lift) : LiftCmd(lift, Lift.groundPos)
    class LiftLowCmd(lift: Lift) : LiftCmd(lift, Lift.lowPos)
    class LiftMidCmd(lift: Lift) : LiftCmd(lift, Lift.midPos)
    class LiftHighCmd(lift: Lift) : LiftCmd(lift, Lift.highPos)
}