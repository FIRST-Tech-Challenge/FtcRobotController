package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class DepositSequence(
    lift: Lift,
    arm : Arm,
    claw: Claw,
    armAngle : Double,
    LiftHeight : Double
) : ParallelGroup(
    InstantCmd({arm.setPos(armAngle)}, arm),
    InstantCmd({lift.setPos(LiftHeight)}, lift),
    ClawCmds.ClawCloseCmd(claw)
)