package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds
import teamcode.v1.Robot
import teamcode.v1.commands.subsystems.ArmCmds
import teamcode.v1.constants.ArmConstants
import kotlin.math.max

class StackSeq(
    robot : Robot,
) : SequentialGroup(
    LiftCmds.LiftCmd(robot.lift, robot.stackHeight),
    ArmCmds.ArmCmd(robot.arm, ArmConstants.groundPos),
    InstantCmd({ robot.stack = max(robot.stack - 1, 0) })
)