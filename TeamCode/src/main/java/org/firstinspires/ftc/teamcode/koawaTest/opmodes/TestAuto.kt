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
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.auto.AutoRobot
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.HomeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ArmCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.constants.ArmConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.LiftConstants

@Autonomous
class TestAuto : KOpMode() {
    private val robot by lazy { AutoRobot(startPose) }

    private val startPose = Pose(-66.0, -44.0, 180.0.radians)

    private val kN = 0.6
    private val kOmega = 1.0 / 30.0
    private val kF = 4.0
    private val kS = 1.0
    private val epsilon = 1.0

    private lateinit var mainCommand: Cmd

    private fun defaultGVFCmd(path: Path, vararg cmds: Pair<Cmd, Vector>): GVFCmd {
        return GVFCmd(
            robot.drive,
            SimpleGVFController(path, 0.6, 1.0 / 30.0, 4.0, 1.0, 1.0, 2.0)
        )
    }

    private val path1 = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(startPose.x, startPose.y, 0.0),
        Pose(-8.0, -41.0, 65.0.radians)
    )

    private val intakePath = HermitePath(
        DEFAULT_HEADING_CONTROLLER,
        Pose(-8.0, -41.0, 250.0.radians),
        Pose(-12.0, -61.5, 270.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-12.0, -61.5, 90.0.radians),
        Pose(-8.0, -41.0, 65.0.radians)
    )

    private val intakePath2 = HermitePath(
        { 90.0.radians },
        Pose(-8.0, -41.0, 90.0.radians),
        Pose(-12.0, 47.0, 90.0.radians)
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
                SimpleGVFController(path1, 0.6, 35.0, 6.0, 0.7, 5.0, 10.0),
                Pair(DepositSequence(robot.lift, robot.arm, robot.claw, 110.0, LiftConstants.highPos), ProjQuery(
                    Vector(-45.0, -45.0)
                )
                )
            ),
           ClawCmds.ClawOpenCmd(robot.claw),
           WaitCmd(1.0),
            GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 35.0, 4.0, 0.4, 5.0, 10.0),
                Pair(
                    HomeSequence(robot.lift, robot.claw, robot.arm, -50.0), ProjQuery(
                    Vector(-8.0, -41.0)
                )
                )
           ),
           WaitCmd(1.0),
           ClawCmds.ClawCloseCmd(robot.claw),
           WaitCmd(0.5),
           InstantCmd({robot.arm.setPos(50.0)}),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 35.0, 6.0, 0.7, 5.0, 10.0),
               Pair(DepositSequence(robot.lift, robot.arm, robot.claw, 145.0, LiftConstants.highPos), ProjQuery(
                   Vector(-14.0, -58.0)
               )
               )
           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
//           ),
//           WaitCmd(0.5),
//           GVFCmd(
//               robot.drive,
//               SimpleGVFController(intakePath2, 0.6, 20.0, 4.0, 0.7, 5.0, 10.0)
//           )
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