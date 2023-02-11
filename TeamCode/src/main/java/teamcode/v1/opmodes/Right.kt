package teamcode.v1.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.*
import com.asiankoala.koawalib.command.group.ParallelGroup
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
import teamcode.v1.auto.AutoRobot
import teamcode.v1.commands.sequences.AutoDepositSequence
import teamcode.v1.commands.sequences.AutoHomeSequence
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.constants.*
import teamcode.v1.vision.Enums

@Autonomous(preselectTeleOp = "KTeleOp")
class Right : KOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, -38.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-11.0, -32.0, 50.0.radians)
    )

    private val intakePath1 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-11.5, -32.5, 240.0.radians),
        Pose(-14.0, -60.5, 270.0.radians)
    )

    private val intakePath2 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-15.0, -31.5, 270.0.radians),
        Pose(-13.5, -60.0, 270.0.radians)
    )

    private val intakePath3 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-15.0, -31.5, 270.0.radians),
        Pose(-12.5, -59.5, 270.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-14.0, -59.0, 90.0.radians),
        Pose(-14.0, -53.0, 90.0.radians),
        Pose(-24.5, -32.5, 50.0.radians)
    )

    private val leftPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-12.5, -31.5, 270.0.radians),
        Pose(-12.5, -57.5, 270.0.radians)
    )

    private val middlePath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-12.5, -58.5, 90.0.radians),
        Pose(-12.5, -3.0, 90.0.radians),
    )

    private val rightPath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-15.0, -31.5, 270.0.radians),
        Pose(-12.5, -58.5, 270.0.radians)
    )

    override fun mInit() {
        robot.vision.start()
        robot.claw.setPos(ClawConstants.closePos)
        Logger.config = LoggerConfig(
            isLogging = true,
            false,
            isDashboardEnabled = true,
            isTelemetryEnabled = true
        )

        mainCommand = SequentialGroup(
            WaitUntilCmd {opModeState == OpModeState.START},
            InstantCmd({robot.whacker.setPos(WhackerConstants.leftPos)}),
            InstantCmd({robot.lift.setPos(7.0)}),
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.4, 20.0, 12.0, 0.6, 3.0, 3.0),
                Pair(
                    AutoDepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, robot.whacker, 137.0, LiftConstants.highPos, GuideConstants.depositPos, WhackerConstants.rightPos), ProjQuery(
                        Vector(-60.0, -40.0)
                    )
                )
            ),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, ArmConstants.groundPos, 5.0, GuideConstants.telePos),
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(intakePath1, 0.6, 20.0, 12.0, 0.4, 1.5, 1.5)
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.1),
            InstantCmd({robot.lift.setPos(11.0)}),
            WaitCmd(0.1),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.4, 30.0, 22.0, 0.7, 3.0, 3.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 145.0, LiftConstants.midPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-15.0, -59.0)
                    )
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, ArmConstants.groundPos, 4.0, GuideConstants.telePos),
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(intakePath2, 0.6, 20.0, 12.0, 0.4, 1.5, 1.5)
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.1),
            InstantCmd({robot.lift.setPos(9.5)}),
            WaitCmd(0.1),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.4, 30.0, 22.0, 0.7, 3.0, 3.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 145.0, LiftConstants.midPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-15.0, -59.0)
                    )
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, ArmConstants.groundPos, 3.0, GuideConstants.telePos),
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(intakePath2, 0.6, 20.0, 12.0, 0.4, 1.5, 1.5)
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.1),
            InstantCmd({robot.lift.setPos(9.0)}),
            WaitCmd(0.1),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.4, 30.0, 22.0, 0.7, 3.0, 3.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 145.0, LiftConstants.midPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-15.0, -59.0)
                    )
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, ArmConstants.groundPos, 1.5, GuideConstants.telePos),
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(intakePath3, 0.6, 20.0, 12.0, 0.4, 1.5, 1.5)
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.1),
            InstantCmd({robot.lift.setPos(8.5)}),
            WaitCmd(0.1),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.4, 30.0, 22.0, 0.7, 3.0, 3.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 145.0, LiftConstants.midPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-15.0, -59.0)
                    )
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, ArmConstants.groundPos, 0.5, GuideConstants.telePos),
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(intakePath3, 0.6, 20.0, 12.0, 0.4, 1.5, 1.5)
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.1),
            InstantCmd({robot.lift.setPos(8.0)}),
            WaitCmd(0.1),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.4, 30.0, 22.0, 0.7, 3.0, 3.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 145.0, LiftConstants.midPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-15.0, -59.0)
                    )
                )
            ),
            WaitCmd(0.1),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.1),
            ParallelGroup(
                AutoHomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.autoIntervalPos, -80.0, -1.0, GuideConstants.telePos),
                GVFCmd(robot.drive,
                    SimpleGVFController(leftPath, 0.6, 20.0, 6.0, 0.4, 1.5, 1.5))
            ),
            ChooseCmd(
                GVFCmd(robot.drive,
                    SimpleGVFController(rightPath, 0.6, 20.0, 6.0, 0.4, 1.5, 1.5)),
                ChooseCmd(
                    GVFCmd(robot.drive,
                        SimpleGVFController(middlePath, 0.6, 20.0, 6.0, 0.4, 1.5, 1.5)),
                    WaitCmd(0.1),
                ) { robot.vision.zone == Enums.Zones.MIDDLE },
            ) { robot.vision.zone == Enums.Zones.RIGHT }
        )
        mainCommand.schedule()
    }

    override fun mInitLoop() {
        Logger.addTelemetryData("zone", robot.vision.zone)
        Logger.addTelemetryData("arm pos", robot.arm.motor.pos)
    }

    override fun mStart() {
        robot.vision.stop()
        robot.vision.unregister()
    }

    override fun mLoop() {
        Logger.addTelemetryData("arm pos", robot.arm.motor.pos)
    }
}