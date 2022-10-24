package org.firstinspires.ftc.teamcode.testing.opmodes

import org.firstinspires.ftc.teamcode.testing.Robot
import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.Vector
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.*
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
class TestAuto : KOpMode() {
    private val robot by lazy { Robot(startPose) }

    private val startPose = Pose(-66.0, -36.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private fun defaultGVFCmd(path: Path, vararg cmds: Pair<Cmd, Vector>): GVFCmd {
        return GVFCmd(
            robot.drive,
            SimpleGVFController(path, 0.6, 1.0 / 22.5, 4.0, 0.95, 2.0)
        )
    }

    val path1 = ReversedCubicPath(
        Pose(-66.0, -36.0, 0.0),
        Pose(-24.0, -36.0, 30.0.radians)
    )

//    val path2 = Path(HermiteSplineInterpolator(
//        HeadingController { it.angle + 180.0.radians },
//        Pose(0.0, 0.0, 0.0),
//        Pose(24.0, 24.0, 0.0)
//    ))

    override fun mInit() {

        Logger.config = LoggerConfig(
            isLogging = true,
            false,
            isDashboardEnabled = true,
            isTelemetryEnabled = true
        )

       mainCommand = SequentialGroup(
            WaitUntilCmd { opModeState == OpModeState.LOOP },
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.6, 1.0/22.5, 4.0, 0.95, 2.0)
            )
        )
        mainCommand.schedule()
    }
}