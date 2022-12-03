package org.firstinspires.ftc.teamcode.koawaTest.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.GVFCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
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

    private val startPose = Pose(-66.0, -36.0, 180.0.radians)

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
        Pose(-3.0, -28.0, 30.0.radians)
    )

    private val intakePath = HermitePath(
        { 270.0.radians },
        Pose(-3.0, -28.0, 250.0.radians),
        Pose(-12.0, -66.0, 270.0.radians)
    )

    private val depositPath = HermitePath(
        FLIPPED_HEADING_CONTROLLER,
        Pose(-12.0, -66.0, 90.0.radians),
        Pose(-3.0, -28.0, 30.0.radians)
    )

    private val intakePath2 = HermitePath(
        { 90.0.radians },
        Pose(-3.0, -28.0, 90.0.radians),
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
                SimpleGVFController(path1, 0.6, 30.0, 4.0, 0.7, 5.0, 10.0),
                Pair(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.highPos, LiftConstants.highPos), ProjQuery(
                    Vector(-35.0, -35.0)
                )
                )
            ),
           WaitCmd(0.5),
           ClawCmds.ClawOpenCmd(robot.claw),
            GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0),
                Pair(
                    HomeSequence(robot.lift, robot.claw, robot.arm), ProjQuery(
                    Vector(-10.0, -40.0)
                )
                )
           ),
           WaitCmd(0.5),
           ClawCmds.ClawCloseCmd(robot.claw),
           ArmCmds.ArmLowCmd(robot.arm),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0),
//               Pair(DepositSequence(robot, ArmConstants.highPos, LiftConstants.highPos), ProjQuery(
//                   Vector(-8.0, -40.0)
//               )
//               )
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath, 0.6, 30.0, 4.0, 0.4, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(depositPath, 0.5, 25.0, 2.0, 0.7, 5.0, 10.0)
           ),
           WaitCmd(0.5),
           GVFCmd(
               robot.drive,
               SimpleGVFController(intakePath2, 0.6, 20.0, 4.0, 0.7, 5.0, 10.0)
           )
        )
        mainCommand.schedule()
    }
}