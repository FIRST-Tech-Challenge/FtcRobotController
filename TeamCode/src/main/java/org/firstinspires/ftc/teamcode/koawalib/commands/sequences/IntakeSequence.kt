package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.RobotState
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw

class IntakeSequence(
    claw: Claw,
) : SequentialGroup(
    WaitUntilCmd(claw::readyToGrab),
    ClawCmds.ClawCloseCmd(claw),
    InstantCmd(RobotState::nextState)
)