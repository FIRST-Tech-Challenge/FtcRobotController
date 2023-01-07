package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.commands.*
import com.asiankoala.koawalib.command.group.SequentialGroup
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.Vector
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.path.FLIPPED_HEADING_CONTROLLER
import com.asiankoala.koawalib.path.HermitePath
import com.asiankoala.koawalib.path.ProjQuery
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

@Autonomous(preselectTeleOp = "KTeleOp")
class ParkRedFar : AutoOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, -36.0, 180.0.radians)

    private lateinit var mainCommand: Cmd

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-24.0, -36.0, 0.0),
        Pose(-6.0, -27.0, 45.0.radians)
    )

    private val leftPath = HermitePath(
        { 180.0.radians },
        Pose(-6.0, -27.0, 180.0.radians),
        Pose(-16.0, -58.0, 180.0.radians),
        Pose(-18.0, -58.0, 180.0.radians),
        Pose(-20.0, -58.0, 180.0.radians)
    )

    private val middlePath = HermitePath(
        { 180.0.radians },
        Pose(-6.0, -27.0, 180.0.radians),
        Pose(-10.0, -35.0, 180.0.radians),
        Pose(-14.0, -35.0, 180.0.radians),
        Pose(-20.0, -35.0, 180.0.radians)
    )

    private val rightPath = HermitePath(
        { 180.0.radians },
        Pose(-6.0, -27.0, 180.0.radians),
        Pose(-16.0, -11.0, 180.0.radians),
        Pose(-18.0, -11.0, 180.0.radians),
        Pose(-20.0, -11.0, 180.0.radians)
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
            WaitUntilCmd { opModeState == OpModeState.START },
            GVFCmd(
                robot.drive,
                SimpleGVFController(path1, 0.6, 20.0, 6.0, 0.5, 5.0, 10.0),
                Pair(
                    DepositSequence(
                        robot.lift,
                        robot.arm,
                        robot.claw,
                        133.0,
                        LiftConstants.highPos
                    ), ProjQuery(
                        Vector(-45.0, -45.0)
                    )
                )
            ),
            WaitCmd(1.0),
            ClawCmds.ClawOpenCmd(robot.claw),
            WaitCmd(0.5),
            HomeSequence(robot.lift, robot.claw, robot.arm, -100.0),
            WaitCmd(0.5),
            ChooseCmd(
                GVFCmd(
                    robot.drive,
                    SimpleGVFController(rightPath, 0.5, 30.0, 6.0, 0.5, 5.0, 10.0)
                ),
                ChooseCmd(
                    GVFCmd(
                        robot.drive,
                        SimpleGVFController(middlePath, 0.5, 30.0, 6.0, 0.5, 5.0, 10.0)
                    ),
                    GVFCmd(
                        robot.drive,
                        SimpleGVFController(leftPath, 0.5, 30.0, 6.0, 0.5, 5.0, 10.0)
                    ),
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