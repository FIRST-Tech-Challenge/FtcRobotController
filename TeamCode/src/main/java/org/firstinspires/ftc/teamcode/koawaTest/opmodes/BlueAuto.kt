package org.firstinspires.ftc.teamcode.koawaTest.opmodes

import com.asiankoala.koawalib.command.KOpMode
import org.firstinspires.ftc.teamcode.koawalib.Robot
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.HermitePath
import com.asiankoala.koawalib.path.gvf.Constraints
import com.asiankoala.koawalib.path.gvf.MotionProfiledGVFController
import com.asiankoala.koawalib.path.gvf.SimpleGVFController
import com.asiankoala.koawalib.util.OpModeState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.koawalib.constants.GVFConfig

@Autonomous
@Disabled
class BlueAuto : KOpMode() {

    private val startPose = Pose(-59.0, -36.0, 0.0.radians)
    private val robot by lazy { Robot(startPose) }

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        { 0.0 },
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-9.0, -36.0, 0.0.radians)
    )

    private val leftPath = HermitePath(
        {180.0.radians},
        Pose(-3.0, -28.0, 180.0.radians),
        Pose(-12.0, -58.0, 180.0.radians)
    )

    private val middlePath = HermitePath(
        {180.0.radians},
        Pose(-3.0, -28.0, 180.0.radians),
        Pose(-12.0, -35.0, 180.0.radians)
    )

    private val rightPath = HermitePath(
        {180.0.radians},
        Pose(-3.0, -28.0, 180.0.radians),
        Pose(-12.0, -11.0, 180.0.radians)
    )

    private val motionProfiledGVFController = MotionProfiledGVFController(
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

    private val parkRight = SimpleGVFController(
                    rightPath,
                    0.6,
                    30.0,
                    4.0,
                    0.7,
                    2.0,
                    5.0
                )

    private val parkMiddle = SimpleGVFController(
        middlePath,
        0.6,
        30.0,
        4.0,
        0.7,
        2.0,
        5.0
    )

    private val parkLeft = SimpleGVFController(
        leftPath,
        0.6,
        30.0,
        4.0,
        0.7,
        2.0,
        5.0
    )


    private val simpleGVFController = SimpleGVFController(
        path1,
        0.6,
        30.0,
        4.0,
        0.7,
        2.0,
        5.0
    )

    override fun mInit() {
//        super.mInit()
        Logger.config = LoggerConfig(
            isLogging = true,
            false,
            isDashboardEnabled = true,
            isTelemetryEnabled = true
        )

        mainCommand = SequentialGroup(
            WaitUntilCmd { opModeState == OpModeState.START },
            WaitUntilCmd(driver.a::isPressed),
            GVFCmd(
                robot.drive,
                motionProfiledGVFController
            ),
//            ChooseCmd(
//                GVFCmd(robot.drive, parkRight),
//                ChooseCmd(
//                    GVFCmd(robot.drive, parkMiddle),
//                    GVFCmd(robot.drive, parkLeft),
//                ) { tagOfInterest!!.id == MIDDLE },
//            ) { tagOfInterest!!.id == RIGHT }
        )
        mainCommand.schedule()
    }

    override fun mLoop() {
        super.mLoop()
        Logger.addVar("velocity", robot.drive.vel.vec.norm)
        Logger.addVar("target velocity", motionProfiledGVFController.state.v)
    }
}