package teamcode.v1.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.NVector
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.v1.Robot
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.ClawConstants
import teamcode.v1.constants.LiftConstants
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
open class KTeleOp() : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }
    private var slowMode = false

    override fun mInit() {
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleDrive()
        scheduleCycling()
//        scheduleTest()
    }

    private fun scheduleDrive() {
        robot.drive.defaultCommand = object : Cmd() {
            val fastScalars = NVector(0.9, 0.9, 0.75)
            val slowScalars = NVector(0.4, 0.4, 0.4)
            val scalars get() = if(slowMode) slowScalars else fastScalars

            private fun joystickFunction(s: Double, k: Double, x: Double): Double {
                return max(0.0, s * x * (k * x.pow(3) - k + 1)) * x.sign
            }

            override fun execute() {
                val raws = NVector(
                    driver.leftStick.xSupplier.invoke(),
                    -driver.leftStick.ySupplier.invoke(),
                    -driver.rightStick.xSupplier.invoke()
                )

                robot.drive.powers = raws
                    .mapIndexed { i, d -> joystickFunction(scalars[i], 1.0, d) }
                    .asPose
            }

            init {
                addRequirements(robot.drive)
            }
        }

        driver.a.onPress(InstantCmd({ slowMode = !slowMode }))
    }

    private fun scheduleCycling() {
        driver.rightBumper.onPress(HomeSequence(robot.lift, robot.claw, robot.arm, ArmConstants.intervalPos, ArmConstants.groundPos, ClawConstants.intakePos))
        driver.leftBumper.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.highPos, LiftConstants.highPos, ClawConstants.outtakePos))
        driver.leftTrigger.onPress(ClawCmds.ClawCloseCmd(robot.claw))
        driver.dpadUp.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.midPos, LiftConstants.midPos, ClawConstants.outtakePos))
//        driver.x.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.groundPos, LiftConstants.groundPos))
        driver.y.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.lowPos, LiftConstants.lowPos, ClawConstants.outtakePos))
        driver.rightTrigger.onPress(ClawCmds.ClawOpenCmd(robot.claw ))
        gunner.leftBumper.onPress(InstantCmd({robot.lift.setPos(2.0)}))
        gunner.rightBumper.onPress(InstantCmd({robot.arm.setPos(10.0)}))
    }

    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(ArmConstants.highPos)}, robot.arm))
        driver.rightBumper.onPress(InstantCmd({robot.lift.setPos(LiftConstants.highPos)}, robot.lift))
//        driver.leftBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.openPos)}))
//        driver.rightBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.closePos)}))
        driver.a.onPress(InstantCmd({robot.arm.setPos(-10.0)}, robot.arm))
        driver.b.onPress(InstantCmd({robot.lift.setPos(0.0)}, robot.lift))
    }

    override fun mLoop() {
        Logger.addVar("robot vel", robot.hardware.liftLeadMotor.vel)
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("lift pos", robot.hardware.liftLeadMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)
        Logger.addTelemetryData("lift power", robot.hardware.liftLeadMotor.power)
        Logger.addTelemetryData("detect is true?", robot.arm.detect)
        Logger.addTelemetryData("claw pos", ClawConstants.openPos)
    }
}