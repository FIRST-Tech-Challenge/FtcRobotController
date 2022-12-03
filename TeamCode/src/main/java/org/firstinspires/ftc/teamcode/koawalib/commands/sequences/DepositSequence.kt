package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds

class DepositSequence(
    robot: Robot,
    armAngle : Double,
    LiftHeight : Double
) : ParallelGroup(
    InstantCmd({robot.arm.setPos(armAngle)}, robot.arm),
    InstantCmd({robot.lift.setPos(LiftHeight)}, robot.lift),
    ClawCmds.ClawCloseCmd(robot.claw)
)