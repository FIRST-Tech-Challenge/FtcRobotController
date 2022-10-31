package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.asiankoala.koawalib.util.Alliance
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.RobotState
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.IntakeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.ReadySequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.DriveHackCmd

open class KTeleOp(private val alliance: Alliance) : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }

    override fun mInit() {
        scheduleDrive()
        scheduleStrategy()
        scheduleCycling()
    }

    private fun scheduleDrive() {
        robot.drive.defaultCommand = DriveHackCmd(
            robot.drive,
            driver.leftStick,
            driver.rightStick,
            driver.leftTrigger::isToggled,
            driver.a::isToggled,
            alliance
        )
    }

    private fun scheduleStrategy() {
        driver.leftBumper.onPress(InstantCmd(RobotState::incStrat))
        driver.rightBumper.onPress(InstantCmd(RobotState::decStrat))
    }

    private fun scheduleCycling() {
        driver.rightTrigger.onPress(
            InstantCmd({
                +when (RobotState.state) {
                    RobotState.State.INTAKING -> IntakeSequence(robot.claw)
                    RobotState.State.READYING -> ReadySequence(robot)
                    RobotState.State.DEPOSITING -> DepositSequence(robot)
                }
            })
        )
    }

    override fun mLoop() {
        Logger.addTelemetryData("state", RobotState.state)
        Logger.addTelemetryData("strat", RobotState.strategy)
        Logger.addTelemetryData("aimbot", driver.a.isToggled)
        Logger.addTelemetryData("spaceglide", driver.leftTrigger.isToggled)
    }
}