package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

class HomeSequence(
    lift: Lift,
    claw : Claw,
    arm : Arm,
    firstArmAngle : Double,
    secondArmAngle : Double
) : ParallelGroup(
    LiftCmds.LiftHomeCmd(lift),
    ClawCmds.ClawOpenCmd(claw),
    SequentialGroup(
    InstantCmd({arm.setPos(firstArmAngle)}, arm),
    WaitCmd(0.6),
    InstantCmd({arm.setPos(secondArmAngle)}, arm))
)
