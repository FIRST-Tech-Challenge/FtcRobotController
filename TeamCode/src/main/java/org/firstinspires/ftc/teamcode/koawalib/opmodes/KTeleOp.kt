package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
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
//        scheduleTest()
    }

    private fun scheduleDrive() {
//        robot.drive.defaultCommand = DriveHackCmd(
//            robot.drive,
//            driver.leftStick,
//            driver.rightStick,
//            driver.leftTrigger::isToggled,
//            driver.a::isToggled,
//            robot.drive::pose
//        )
        robot.drive.defaultCommand = MecanumCmd(
            robot.drive,
            driver.leftStick.xInverted.yInverted,
            driver.rightStick.xInverted,
            0.9,
            0.9,
            0.9,
            1.0,
            1.0,
            1.0
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

    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(135.0)}, robot.arm))
        driver.rightBumper.onPress(InstantCmd({robot.lift.setPos(14.5)}, robot.lift))
        driver.a.onPress(InstantCmd({robot.arm.setPos(-50.0)}, robot.arm))
        driver.b.onPress(InstantCmd({robot.lift.setPos(-1.0)}, robot.lift))
    }

    override fun mLoop() {
//        Logger.addTelemetryData("state", RobotState.state)
//        Logger.addTelemetryData("strat", RobotState.strategy)
//        Logger.addTelemetryData("aimbot", driver.a.isToggled)
//        Logger.addTelemetryData("spaceglide", driver.leftTrigger.isToggled)
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("lift pos", robot.hardware.liftLeadMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)
        Logger.addTelemetryData("lift power", robot.hardware.liftLeadMotor.power)

    }
}