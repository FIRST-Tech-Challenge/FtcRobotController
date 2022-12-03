package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ArmCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds

class HomeSequence(robot: Robot) : SequentialGroup(
    ParallelGroup(
    LiftCmds.LiftHomeCmd(robot.lift),
    ClawCmds.ClawOpenCmd(robot.claw),
    ),
    WaitCmd(0.1),
    ArmCmds.ArmHomeCmd(robot.arm)
)