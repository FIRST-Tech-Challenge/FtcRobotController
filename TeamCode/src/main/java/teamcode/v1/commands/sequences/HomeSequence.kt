package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.ChooseCmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import teamcode.v1.Robot
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.constants.GuideConstants
class HomeSequence(
    robot : Robot,
    secondArmAngle : Double,
    liftHeight : Double,
    GripPos : Double
) : SequentialGroup(
    ParallelGroup(
        InstantCmd({robot.guide.stopReading()}),
        InstantCmd({robot.guide.setPos(GripPos)}),
    ),
    InstantCmd({robot.arm.setPos(secondArmAngle)}, robot.arm),
    ClawCmds.ClawCloseCmd(robot.claw),
    InstantCmd({robot.lift.setPos(liftHeight)}),
    WaitCmd(0.5),
    ClawCmds.AutoOpenCmd(robot.claw, robot.guide, GuideConstants.telePos)
)

