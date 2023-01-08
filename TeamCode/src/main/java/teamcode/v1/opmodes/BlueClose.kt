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
import org.firstinspires.ftc.teamcode.koawalib.auto.AutoRobot
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.HomeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants
import teamcode.v1.constants.LiftConstants
import org.firstinspires.ftc.teamcode.koawalib.vision.AutoOpMode

@Autonomous
class BlueClose : AutoOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, -36.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-24.0, -33.0, 0.0.radians),
        Pose(-8.0, -23.0, 50.0.radians)
    )

    private val intakePath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-8.0, -23.0, 200.0.radians),
        Pose(-12.0, -53.0, 270.0.radians)
    )

    private val intakePath2 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-8.0, -23.0, 200.0.radians),
        Pose(-12.0, -54.0, 270.0.radians)
    )

    private val intakePath3 = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-8.0, -23.0, 200.0.radians),
        Pose(-12.0, -55.0, 270.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-14.0, -53.0, 90.0.radians),
        Pose(-7.5, -28.5, 50.0.radians)
    )

    private val leftPath = HermitePath(
        {180.0.radians},
        Pose(-7.5, -28.5, 180.0.radians),
        Pose(-20.0, -30.0, 180.0.radians),
        Pose(-20.0, -11.0, 180.0.radians)
    )

    private val middlePath = HermitePath(
        {180.0.radians},
        Pose(-7.5, -28.5, 180.0.radians),
        Pose(-20.0, -30.0, 180.0.radians),
        Pose(-20.0, -35.0, 180.0.radians)
    )

    private val rightPath = HermitePath(
        {180.0.radians},
        Pose(-7.5, -28.5, 180.0.radians),
        Pose(-20.0, -30.0, 180.0.radians),
        Pose(-20.0, -58.0, 180.0.radians)
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
            InstantCmd({robot.arm.setPos(170.0)}),
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.6, 20.0, 6.0, 0.7, 5.0, 10.0),
                Pair(DepositSequence(robot.lift, robot.arm, robot.claw, 145.0, LiftConstants.highPos), ProjQuery(
                    Vector(-60.0, -36.0)
                )
                )
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, 0.0, -36.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath, 0.6, 15.0, 4.0, 0.3, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.25),
            InstantCmd({robot.lift.setPos(10.0)}),
            WaitCmd(0.25),
            DepositSequence(robot.lift, robot.arm, robot.claw, 153.0, LiftConstants.highPos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, 0.0, -41.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath2, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.25),
            InstantCmd({robot.lift.setPos(10.0)}),
            WaitCmd(0.25),
            DepositSequence(robot.lift, robot.arm, robot.claw, 153.0, LiftConstants.highPos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, 0.0, -46.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath2, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.25),
            InstantCmd({robot.lift.setPos(10.0)}),
            WaitCmd(0.25),
            DepositSequence(robot.lift, robot.arm, robot.claw, 153.0, LiftConstants.highPos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm, 0.0, -51.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath3, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.25),
            InstantCmd({robot.lift.setPos(10.0)}),
            WaitCmd(0.25),
            DepositSequence(robot.lift, robot.arm, robot.claw, 153.0, LiftConstants.highPos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
            HomeSequence(robot.lift, robot.claw, robot.arm,0.0,  -56.5),
            GVFCmd(
                robot.drive,
                SimpleGVFController(intakePath3, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
            ),
            WaitCmd(0.5),
            ClawCmds.ClawCloseCmd(robot.claw),
            WaitCmd(0.25),
            InstantCmd({robot.lift.setPos(10.0)}),
            WaitCmd(0.25),
            DepositSequence(robot.lift, robot.arm, robot.claw, 153.0, LiftConstants.highPos),
            GVFCmd(
                robot.drive,
                SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
            ),
            WaitCmd(0.5),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.25),
//            ChooseCmd(
//                GVFCmd(robot.drive,
//                    SimpleGVFController(rightPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0)),
//                ChooseCmd(
//                    GVFCmd(robot.drive,
//                        SimpleGVFController(middlePath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0)),
//                    GVFCmd(robot.drive, SimpleGVFController(leftPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0)),
//                ) { tagOfInterest!!.id == MIDDLE },
//            ) { tagOfInterest!!.id == RIGHT },
            HomeSequence(robot.lift, robot.claw, robot.arm, 0.0, -100.0)
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