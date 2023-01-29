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
import teamcode.v1.auto.AutoRobot
import teamcode.v1.commands.sequences.AutoDepositSequence
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.commands.subsystems.GuideCmds
import teamcode.v1.constants.*
import teamcode.v1.vision.AutoOpMode

@Autonomous(preselectTeleOp = "KTeleOp")
class Left : AutoOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, 40.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-10.0, 33.5, 310.0.radians)
    )

    private val intakePath1 = HermitePath(
            DEFAULT_HEADING_CONTROLLER,
            Pose(-12.0, 33.5, 120.0.radians),
            Pose(-15.5, 62.25, 90.0.radians)
    )

    private val intakePath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-12.0, 33.5, 120.0.radians),
        Pose(-15.5, 61.25, 90.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-16.0, 64.0, 270.0.radians),
        Pose(-16.0, 56.0, 275.0.radians),
        Pose(-6.5, 33.0, 310.0.radians)
    )

    private val leftPath = HermitePath(
            DEFAULT_HEADING_CONTROLLER,
            Pose(-12.0, 33.5, 120.0.radians),
            Pose(-15.5, 63.0, 90.0.radians)
    )

    private val middlePath = HermitePath(
        {180.0.radians},
        Pose(-8.0, 33.5, 180.0.radians),
        Pose(-16.0, 39.0, 180.0.radians),
        Pose(-18.0, 39.0, 180.0.radians),
        Pose(-20.0, 39.0, 180.0.radians)
    )

    private val rightPath = HermitePath(
        {180.0.radians},
        Pose(-12.0, 33.5, 180.0.radians),
        Pose(-20.0, 15.0, 180.0.radians)
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
            InstantCmd({robot.whacker.setPos(WhackerConstants.rightPos)}),
            InstantCmd({robot.lift.setPos(7.0)}),
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.6, 20.0, 12.0, 0.6, 3.0, 5.0),
                Pair(
                    AutoDepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, robot.whacker, 145.0, LiftConstants.highPos, GuideConstants.depositPos, WhackerConstants.leftPos), ProjQuery(
                        Vector(-60.0, 40.0)
                    )
                )
            ),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.5),
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, 4.0, GuideConstants.telePos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath1, 0.6, 20.0, 8.0, 0.4, 3.0, 5.0)
            ),
            WaitCmd(0.25),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.5),
            InstantCmd({robot.lift.setPos(11.0)}),
            GuideCmds.GuideDepositCmd(robot.guide),
            WaitCmd(0.25),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 15.0, 0.4, 3.0, 5.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 160.0, LiftConstants.highPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-14.0, 66.0)
                    )
                )
            ),
            WaitCmd(0.25),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, 3.0, GuideConstants.telePos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath, 0.6, 20.0, 8.0, 0.4, 3.0, 5.0)
            ),
            WaitCmd(0.25),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.5),
            InstantCmd({robot.lift.setPos(11.0)}),
            GuideCmds.GuideDepositCmd(robot.guide),
            WaitCmd(0.25),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 15.0, 0.4, 3.0, 5.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 160.0, LiftConstants.highPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-14.0, 66.0)
                    )
                )
            ),
            WaitCmd(0.25),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, 2.0, GuideConstants.telePos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath, 0.6, 20.0, 8.0, 0.4, 3.0, 5.0)
            ),
            WaitCmd(0.25),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.5),
            InstantCmd({robot.lift.setPos(11.0)}),
            GuideCmds.GuideDepositCmd(robot.guide),
            WaitCmd(0.25),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 15.0, 0.4, 3.0, 5.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 160.0, LiftConstants.highPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-14.0, 66.0)
                    )
                )
            ),
            WaitCmd(0.25),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, 1.0, GuideConstants.telePos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath, 0.6, 20.0, 8.0, 0.4, 3.0, 5.0)
            ),
            WaitCmd(0.25),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.5),
            InstantCmd({robot.lift.setPos(11.0)}),
            GuideCmds.GuideDepositCmd(robot.guide),
            WaitCmd(0.25),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 15.0, 0.4, 3.0, 5.0),
                Pair(
                    DepositSequence(robot.lift, robot.arm, robot.claw, robot.guide, 160.0, LiftConstants.highPos, GuideConstants.depositPos), ProjQuery(
                        Vector(-14.0, 66.0)
                    )
                )
            ),
            WaitCmd(0.25),
            ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, robot.guide, ArmConstants.intervalPos, ArmConstants.groundPos, -1.0, GuideConstants.telePos),
                ClawCmds.ClawOpenCmd(robot.claw, robot.guide, GuideConstants.telePos),
            ChooseCmd(
                GVFCmd(robot.drive,
                    SimpleGVFController(rightPath, 0.5, 30.0, 6.0, 0.4, 3.0, 5.0)),
                ChooseCmd(
                    GVFCmd(robot.drive,
                        SimpleGVFController(middlePath, 0.5, 30.0, 6.0, 0.3, 3.0, 5.0)),
                    GVFCmd(robot.drive, SimpleGVFController(leftPath,0.5, 30.0, 6.0, 0.3, 3.0, 5.0)),
                ) { tagOfInterest!!.id == MIDDLE },
            ) { tagOfInterest!!.id == RIGHT }
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