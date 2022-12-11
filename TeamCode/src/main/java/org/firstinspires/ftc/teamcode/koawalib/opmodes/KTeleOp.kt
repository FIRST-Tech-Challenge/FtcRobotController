package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.HomeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.constants.ArmConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.LiftConstants

@TeleOp
open class KTeleOp() : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }

    override fun mInit() {
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleDrive()
        scheduleCycling()
//        scheduleTest()
    }

    private fun scheduleDrive() {
        robot.drive.defaultCommand = MecanumCmd(
            robot.drive,
            driver.leftStick,
            driver.rightStick,
            0.9,
            0.7,
            0.5,
        )
    }

    private fun scheduleCycling() {
        driver.rightBumper.onPress(HomeSequence(robot.lift, robot.claw, robot.arm, ArmConstants.homePos))
        driver.leftBumper.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.highPos, LiftConstants.highPos))
        driver.leftTrigger.onPress(ClawCmds.ClawCloseCmd(robot.claw))
        driver.dpadUp.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.midPos, LiftConstants.midPos))
        driver.x.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.groundPos, LiftConstants.groundPos))
        driver.y.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.lowPos, LiftConstants.lowPos))
        driver.b.onPress(ClawCmds.ClawOpenCmd(robot.claw))
        gunner.leftBumper.onPress(InstantCmd({robot.lift.setPos(-17.0)}))
        gunner.rightBumper.onPress(InstantCmd({robot.arm.setPos(250.0)}))
    }

    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(135.0)}, robot.arm))
        driver.rightBumper.onPress(InstantCmd({robot.lift.setPos(14.5)}, robot.lift))
//        driver.leftBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.openPos)}))
//        driver.rightBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.closePos)}))
        driver.a.onPress(InstantCmd({robot.arm.setPos(-50.0)}, robot.arm))
        driver.b.onPress(InstantCmd({robot.lift.setPos(-1.0)}, robot.lift))
    }

    override fun mLoop() {
        Logger.addVar("robot vel", robot.hardware.liftLeadMotor.vel)
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("lift pos", robot.hardware.liftLeadMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)
        Logger.addTelemetryData("lift power", robot.hardware.liftLeadMotor.power)

    }
}