package org.firstinspires.ftc.teamcode.testing.opmodes

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.testing.Robot
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
import com.asiankoala.koawalib.path.gvf.Constraints
import com.asiankoala.koawalib.path.gvf.MotionProfiledGVFController
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline

@Autonomous
class BlueAuto : KOpMode() {

    private val startPose = Pose(-59.0, -36.0, 0.0.radians)
    private val robot by lazy { Robot(startPose) }

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-9.0, -36.0, 0.0.radians)
    )

    val motionProfiledGVFController = MotionProfiledGVFController(
        path1,
        0.6,
        2.0,
        2.0,
        Constraints(30.0, 30.0),
        GVFConfig.kOmega,
        GVFConfig.kStatic,
        GVFConfig.kV,
        GVFConfig.kA,
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
            WaitUntilCmd(driver.a::isPressed),
            GVFCmd(
                robot.drive,
                motionProfiledGVFController
            )
        )
        mainCommand.schedule()
    }

    override fun mLoop() {
        Logger.addVar("velocity", robot.drive.vel.vec.norm)
        Logger.addVar("target velocity", motionProfiledGVFController.state.v)
    }
}