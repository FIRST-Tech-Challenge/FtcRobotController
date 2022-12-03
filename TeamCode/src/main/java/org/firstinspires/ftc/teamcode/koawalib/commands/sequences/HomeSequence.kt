package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ArmCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class HomeSequence(
    lift: Lift,
    claw : Claw,
    arm : Arm
) : SequentialGroup(
    ParallelGroup(
    LiftCmds.LiftHomeCmd(lift),
    ClawCmds.ClawOpenCmd(claw),
    ),
    WaitCmd(0.1),
    ArmCmds.ArmHomeCmd(arm)
)