package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.Vector
import com.asiankoala.koawalib.math.angleWrap
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.DEFAULT_HEADING_CONTROLLER
import com.asiankoala.koawalib.path.FLIPPED_HEADING_CONTROLLER
import com.asiankoala.koawalib.path.HermitePath
import com.asiankoala.koawalib.path.Path
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.Alliance
import com.asiankoala.koawalib.util.OpModeState
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.CmdChooser
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.IntakeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.ReadySequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds

open class KAuto(
    alliance: Alliance,
    close: Boolean,
) : KOpMode() {
    private val startPose = Pose(
        Vector(-66.0, -36.0).choose(alliance, close),
        close.choose(0.0, 180.0.radians)
    )
    private val robot by lazy { Robot(startPose) }
    private val kN = 0.6
    private val kOmega = 1.0 / 30.0.radians
    private val kF = 4.0
    private val kS = 1.0
    private val epsilon = 1.0

    private val initialPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-66.0, -36.0, 0.0),
        Pose(-3.0, -28.0, 30.0.radians)
    ).choose(alliance, close)

    private val intakePath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-3.0, -28.0, 210.0.radians),
        Pose(-12.0, -66.0, 270.0.radians)
    ).choose(alliance, close)

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-12.0, -66.0, 90.0.radians),
        Pose(-3.0, -28.0, 30.0.radians)
    ).choose(alliance, close)

    private val initialReadyP = Pair(
        ReadySequence(robot), Vector(-24.0, -36.0).choose(alliance, close)
    )

    private val intakeP = Pair(
        IntakeSequence(robot.claw), Vector(-12.0, -43.0).choose(alliance, close)
    )

    private val readyP = Pair(
        ReadySequence(robot), Vector(-12.0, -36.0).choose(alliance, close)
    )

    private val depositP = Pair(
        DepositSequence(robot), Vector(-5.0, -30.0).choose(alliance, close)
    )

    private fun getGVFCmd(path: Path, vararg cmds: Pair<Cmd, Vector>) =
        GVFCmd(robot.drive, SimpleGVFController(path, kN, kOmega, kF, kS, epsilon), *cmds)

    override fun mInit() {
        +SequentialGroup(
            ClawCmds.ClawCloseCmd(robot.claw),
            CmdChooser.homeCmd(robot),
            WaitUntilCmd { opModeState == OpModeState.START },
            getGVFCmd(initialPath, initialReadyP, depositP),
            *List(5) {
                listOf(
                    getGVFCmd(intakePath, intakeP),
                    getGVFCmd(depositPath, readyP, depositP)
                )
            }.flatten().toTypedArray()
        )
    }

    companion object {
        private fun <T> Boolean.choose(a: T, b: T) = if (this) a else b
        private fun <T> Alliance.choose(a: T, b: T) = (this == Alliance.BLUE).choose(a, b)

        private fun Vector.checkFlipY(alliance: Alliance) = alliance.choose(this, Vector(-x, y))
        private fun Vector.checkFlipX(close: Boolean) = close.choose(this, Vector(x, -y))
        private fun Vector.choose(alliance: Alliance, close: Boolean) =
            this.checkFlipY(alliance).checkFlipX(close)

        private fun HermitePath.checkFlipY(alliance: Alliance) = alliance.choose(
            this,
            this.map(FLIPPED_HEADING_CONTROLLER) {
                Pose(
                    -it.x,
                    it.y,
                    (180.0.radians - it.heading).angleWrap
                )
            }
        )

        private fun HermitePath.checkFlipX(close: Boolean) = close.choose(
            this,
            this.map(DEFAULT_HEADING_CONTROLLER) {
                Pose(
                    it.x,
                    -it.y,
                    -it.heading
                )
            }
        )

        private fun HermitePath.choose(alliance: Alliance, close: Boolean) =
            this.checkFlipY(alliance).checkFlipX(close)
    }
}