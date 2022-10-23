package org.firstinspires.ftc.teamcode.koawalib.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.RobotState
import org.firstinspires.ftc.teamcode.koawalib.commands.CmdChooser

class ReadySequence(robot: Robot) : SequentialGroup(
    CmdChooser.double(robot),
    InstantCmd(RobotState::nextState)
)