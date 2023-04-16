package teamcode.v1.commands.subsystems

import com.asiankoala.koawalib.command.commands.ChooseCmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.Robot
import teamcode.v1.constants.ClawConstants
import teamcode.v1.subsystems.Claw
import teamcode.v1.subsystems.Guide

class ClawCmds {
    open class ClawCmd(claw: Claw, pos: Double) : InstantCmd({ claw.setPos(pos) }, claw)
//bruh
    class ClawCloseCmd(claw: Claw) : ClawCmd(claw, ClawConstants.closePos)

//    class ClawSmartCmd(claw: Claw) : SequentialGroup(
//        ClawStartReadingCmd(claw),
//        WaitUntilCmd { claw.lastRead < ClawConstants.distanceThreshold },
//        ClawStopReadingCmd(claw),
//        ClawCmd(claw, ClawConstants.closePos),
//    )
    class AutoOpenCmd(claw: Claw, guide : Guide, GripPos: Double) : SequentialGroup(
        ClawCmd(claw, ClawConstants.openPos),
        InstantCmd({guide.setPos(GripPos)})
    )

    class ClawOpenCmd(robot : Robot, GripPos: Double) : SequentialGroup(
        WaitCmd(0.3),
        ParallelGroup(
        InstantCmd({ robot.lift.setPos(robot.lift.leadMotor.pos + 3.0) }),
        InstantCmd({robot.isLifted = false}),
        ClawCmd(robot.claw, ClawConstants.openPos),
        InstantCmd({robot.guide.setPos(GripPos)})
        )
    )
//    class ClawStartReadingCmd(claw: Claw) : InstantCmd(claw::startReading)
//    class ClawStopReadingCmd(claw: Claw) : InstantCmd(claw::stopReading)
}