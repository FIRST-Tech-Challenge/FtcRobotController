package org.firstinspires.ftc.teamcode.testing.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.DEFAULT_HEADING_CONTROLLER
import com.asiankoala.koawalib.path.FLIPPED_HEADING_CONTROLLER
import com.asiankoala.koawalib.path.HermitePath
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.testing.Robot

@Autonomous
class MidAuto : KOpMode() {
    private val robot by lazy { Robot(startPose) }

    private val startPose = Pose(-59.0, -36.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-35.0, -33.0, 35.0.radians)
    )
//
    private val intakePath1 = HermitePath(
        {275.0.radians},
        Pose(-35.0, -33.0, 350.0.radians),
        Pose(-34.0, -57.0, 340.0.radians),
        Pose(-12.0, -52.0, 90.0.radians)
    )

    private val depositPath = HermitePath(
        {120.0.radians},
        Pose(-12.0, -52.0, 110.0.radians),
        Pose(-13.0, -38.0, 120.0.radians)
    )

    private val intakePath2 = HermitePath(
        { 275.0.radians },
        Pose(-13.0, -38.0, 250.0.radians),
        Pose(-12.0, -60.0, 270.0.radians)
    )


    override fun mInit() {
        Logger.config = LoggerConfig(
            isLogging = true,
            false,
            isDashboardEnabled = true,
            isTelemetryEnabled = true
        )

        mainCommand = SequentialGroup(
            WaitUntilCmd {opModeState == OpModeState.START},
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.6, 30.0, 4.0, 0.5, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath1, 0.6, 20.0, 4.0, 0.5, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.6, 20.0, 4.0, 0.5, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath2, 0.6, 20.0, 4.0, 0.4, 5.0, 10.0)
            )
        )
        mainCommand.schedule()
    }
}