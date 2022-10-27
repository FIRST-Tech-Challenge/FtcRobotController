package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.Vector
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.CubicPath
import com.asiankoala.koawalib.path.Path
import com.asiankoala.koawalib.path.ReversedCubicPath
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.CmdChooser
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.IntakeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.ReadySequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds

@Autonomous
class KAuto(startPose : Pose) : KOpMode() {
    private val robot by lazy { Robot(startPose) }
    private val kN = 0.6
    private val kOmega = 1.0 / 30.0.radians
    private val kF = 4.0
    private val kS = 1.0
    private val epsilon = 1.0

    private fun defaultGVFCmd(path: Path, vararg cmds: Pair<Cmd, Vector>): GVFCmd {
        return GVFCmd(
            robot.drive,
            SimpleGVFController(path, kN, kOmega, kF, kS, epsilon),
        )
    }

    override fun mInit() {
        val intakePath = CubicPath(
            Pose(-3.0, -36.0, 210.0.radians),
            Pose(-12.0, -66.0, 270.0.radians)
        )
        val depositPath = ReversedCubicPath(
            Pose(-12.0, -66.0, 90.0.radians),
            Pose(-3.0, -36.0, 30.0.radians)
        )
        val checkpoint = Vector(-12.0, -43.0)
        val intakeP = Pair(IntakeSequence(robot.claw), checkpoint)
        val readyP = Pair(ReadySequence(robot), checkpoint)
        val depositP = Pair(DepositSequence(robot), Vector(-5.0, -30.0))

        + SequentialGroup(
            ClawCmds.ClawOpenCmd(robot.claw),
            CmdChooser.homeCmd(robot),
            WaitUntilCmd { opModeState == OpModeState.START },
            defaultGVFCmd(
                ReversedCubicPath(
                    Pose(-66.0, -36.0, 0.0),
                    Pose(-3.0, -36.0, 30.0.radians)
                ),
                Pair(ReadySequence(robot), Vector(-24.0, -36.0)),
                depositP
            ),
            *List(5) {
                listOf(defaultGVFCmd(intakePath, intakeP),
                    defaultGVFCmd(depositPath, readyP, depositP))
            }.flatten().toTypedArray()
        )
    }
}