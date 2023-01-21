package org.firstinspires.ftc.teamcode.koawalib.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import teamcode.v1.constants.LiftConstants
import teamcode.v1.subsystems.Lift

class LiftCmds {
    open class LiftCmd(lift: Lift, pos: Double) : InstantCmd({ lift.setPos(pos) }, lift)

    class LiftHomeCmd(lift: Lift) : LiftCmd(lift, LiftConstants.homePos)
    class LiftGroundCmd(lift: Lift) : LiftCmd(lift, LiftConstants.groundPos)
    class LiftLowCmd(lift: Lift) : LiftCmd(lift, LiftConstants.lowPos)
    class LiftMidCmd(lift: Lift) : LiftCmd(lift, LiftConstants.midPos)
    class LiftHighCmd(lift: Lift) : LiftCmd(lift, LiftConstants.highPos)
}