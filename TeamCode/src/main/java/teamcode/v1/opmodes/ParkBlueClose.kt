package teamcode.v1.opmodes

import com.asiankoala.koawalib.command.commands.*
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import teamcode.v1.auto.AutoRobot
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.commands.subsystems.GuideCmds
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.ClawConstants
import teamcode.v1.constants.GuideConstants
import teamcode.v1.constants.LiftConstants
import teamcode.v1.vision.AutoOpMode

@Disabled
@Autonomous(preselectTeleOp = "KTeleOp")
class ParkBlueClose : AutoOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, 40.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-45.0, 40.0, 0.0),
        Pose(-17.0, 40.0, 0.0.radians)
    )

    private val leftPath = HermitePath(
        {180.0.radians},
        Pose(-17.0, 40.0, 180.0.radians),
        Pose(-17.0, 66.0, 180.0.radians),
    )

    private val middlePath = HermitePath(
        {180.0.radians},
        Pose(-17.0, 40.0, 180.0.radians),
        Pose(-17.0, 40.0, 180.0.radians),
        Pose(-17.0, 40.0, 180.0.radians),
        Pose(-17.0, 40.0, 180.0.radians)
    )

    private val rightPath = HermitePath(
        {180.0.radians},
        Pose(-10.0, 40.0, 180.0.radians),
        Pose(-16.0, 20.0, 180.0.radians),
        Pose(-18.0, 20.0, 180.0.radians),
        Pose(-20.0, 20.0, 180.0.radians)
    )

    override fun mInit() {
        super.mInit()
        robot.claw.setPos(ClawConstants.closePos)
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
                SimpleGVFController(path1, 0.6, 30.0, 6.0, 0.7, 5.0, 10.0),
            ),
            WaitCmd(1.0),
            ChooseCmd(
                GVFCmd(robot.drive,
                    SimpleGVFController(rightPath, 0.5, 30.0, 6.0, 0.6, 5.0, 10.0)),
                ChooseCmd(
                    GVFCmd(robot.drive,
                        SimpleGVFController(middlePath, 0.5, 30.0, 6.0, 0.6, 5.0, 10.0)),
                    GVFCmd(robot.drive, SimpleGVFController(leftPath, 0.5, 30.0, 6.0, 0.6, 5.0, 10.0)),
                ) { tagOfInterest!!.id == MIDDLE },
            ) { tagOfInterest!!.id == RIGHT },
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, -1.0, GuideConstants.telePos),
        )
        mainCommand.schedule()
    }

    override fun mLoop() {
        super.mLoop()
        Logger.addTelemetryData("arm pos", robot.arm.motor.pos)
    }

    override fun mStop() {
        super.mStop()
        ClawCmds.ClawCloseCmd(robot.claw)
    }
}