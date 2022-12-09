package org.firstinspires.ftc.teamcode.koawaTest.opmodes

import com.asiankoala.koawalib.command.KOpMode
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
import org.firstinspires.ftc.teamcode.koawalib.constants.LiftConstants

@Autonomous
class TestAuto : KOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, -36.0, 180.0.radians)

    private lateinit var mainCommand: Cmd


    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-24.0, -36.0, 0.0),
        Pose(-9.0, -27.0, 45.0.radians)
    )

    private val intakePath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-9.0, -27.0, 200.0.radians),
        Pose(-14.0, -54.0, 270.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-14.0, -54.0, 90.0.radians),
        Pose(-7.0, -30.0, 50.0.radians)
    )

    override fun mInit() {
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
                SimpleGVFController(path1, 0.6, 20.0, 6.0, 0.6, 5.0, 10.0),
                Pair(DepositSequence(robot.lift, robot.arm, robot.claw, 135.0, LiftConstants.highPos), ProjQuery(
                    Vector(-45.0, -45.0)
                )
                )
            ),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(0.5),
           HomeSequence(robot.lift, robot.claw, robot.arm, -30.0),
            GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           DepositSequence(robot.lift, robot.arm, robot.claw, 160.0, LiftConstants.highPos),
           WaitCmd(0.1),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
           ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(0.5),
           HomeSequence(robot.lift, robot.claw, robot.arm, -35.0),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           DepositSequence(robot.lift, robot.arm, robot.claw, 160.0, LiftConstants.highPos),
           WaitCmd(0.1),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
           ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(0.5),
           HomeSequence(robot.lift, robot.claw, robot.arm, -40.0),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           DepositSequence(robot.lift, robot.arm, robot.claw, 160.0, LiftConstants.highPos),
           WaitCmd(0.1),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
           ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(0.5),
           HomeSequence(robot.lift, robot.claw, robot.arm, -45.0),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           DepositSequence(robot.lift, robot.arm, robot.claw, 160.0, LiftConstants.highPos),
           WaitCmd(0.1),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
           ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(0.5),
           HomeSequence(robot.lift, robot.claw, robot.arm, -50.0),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 20.0, 4.0, 0.3, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           DepositSequence(robot.lift, robot.arm, robot.claw, 160.0, LiftConstants.highPos),
           WaitCmd(0.1),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 20.0, 6.0, 0.5, 5.0, 10.0),
           ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
        )
        mainCommand.schedule()
    }

    override fun mLoop() {
        Logger.addTelemetryData("arm pos", robot.arm.motor.pos)
    }

    override fun mStop() {
        ClawCmds.ClawCloseCmd(robot.claw)
    }
}