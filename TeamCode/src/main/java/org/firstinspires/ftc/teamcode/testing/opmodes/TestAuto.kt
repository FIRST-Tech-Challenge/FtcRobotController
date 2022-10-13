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
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.CubicPath
import com.asiankoala.koawalib.path.QuinticPath
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
class TestAuto : KOpMode() {

    private val startPose = Pose(-59.0, -10.0, 0.0.radians)
    //    private val startPose2 = Pose(-16.0, -62.0, 150.0.radians)
//    private val startPose3 = Pose(-3.0, -35.0, 245.0.radians)
    private val robot by lazy { Robot(startPose) }

    private lateinit var mainCommand: Cmd

    val path1 = CubicPath(
        startPose,
        Pose(-10.0,-35.0, 270.0.radians),
        Pose(-30.0,-60.0, 180.0.radians),
        Pose(-59.0, -10.0, 90.0.radians)
    )


//    val path2 = PathBuilder(startPose2.toPose2d())
//        .splineTo(Vector2d(-3.0, -35.0), 65.0.radians)
//        .build()
//
//    val path3 = PathBuilder(startPose3.toPose2d())
//        .splineTo(Vector2d(-12.0, -62.0), 270.0.radians)
//        .build()

    override fun mInit() {
        Logger.config = LoggerConfig(
            isLogging = true,
            true,
            isDashboardEnabled = true,
            isTelemetryEnabled = true
        )


        mainCommand = SequentialGroup(
            WaitUntilCmd { opmodeState == OpModeState.LOOP },
            GVFCmd(
                robot.drive,
                path1,
                0.6,
                1.0 / 22.5,
                4.0,
                0.95,
                2.0
            ),
//            GVFCmd(
//                robot.drive,
//                path2,
//                0.7,
//                1.0 / 25.0,
//                4.0,
//                0.8,
//                2.0
//            ),
//            GVFCmd(
//                robot.drive,
//                path3,
//                0.7,
//                1.0 / 25.0,
//                4.0,
//                0.8,
//                2.0
//            )
        )
        mainCommand.schedule()
    }
}