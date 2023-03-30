package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.ChooseCmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.Robot
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.constants.ClawConstants

class DepositSequence(
    robot : Robot,
    armAngle : Double,
    LiftHeight : Double,
    GripPos : Double,
) : SequentialGroup(
    ClawCmds.ClawCmd(robot.claw, ClawConstants.closePos),
    InstantCmd({robot.arm.setPos(armAngle)}, robot.arm),
    WaitCmd(0.3),
    InstantCmd({robot.lift.setPos(LiftHeight)}, robot.lift),
    WaitCmd(0.1),
    InstantCmd({robot.guide.startReading()}),
    InstantCmd({robot.guide.setPos(GripPos)}),
    WaitUntilCmd{ robot.guide.lastRead < 60.0 || robot.guide.lastReadTwo < 60.0 },
    ChooseCmd(InstantCmd({robot.lift.setPos(LiftHeight - 3.0)}), WaitCmd(0.0)) { LiftHeight - 3.0 > 0.0 },
    InstantCmd({robot.isLifted = true}),
    InstantCmd({robot.guide.stopReading()})
)