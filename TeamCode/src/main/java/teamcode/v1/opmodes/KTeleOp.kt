package teamcode.v1.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.NVector
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.v1.Robot
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.constants.*
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
open class KTeleOp : KOpMode(photonEnabled = false) {
    private val robot by lazy { Robot(Pose(-66.0, 40.0, 180.0.radians)) }
    private var slowMode = false

    override fun mInit() {
        InstantCmd({robot.lift.setPos(1.0)})
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleDrive()
        scheduleCycling()
//        scheduleTest()
    }

    private fun scheduleDrive() {
        robot.drive.defaultCommand = object : Cmd() {
            val fastScalars = NVector(1.0, 1.0, 0.75)
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
        driver.rightBumper.onPress(HomeSequence(robot, ArmConstants.groundPos, 0.0, 0.13))
        driver.leftBumper.onPress(DepositSequence(robot, ArmConstants.highPos, LiftConstants.highPos, GuideConstants.depositPos))
        driver.leftTrigger.onPress(ClawCmds.ClawCloseCmd(robot.claw))
        driver.dpadUp.onPress(DepositSequence(robot, ArmConstants.midPos, LiftConstants.midPos, GuideConstants.depositPos))
        driver.y.onPress(DepositSequence(robot, ArmConstants.lowPos, LiftConstants.lowPos, GuideConstants.lowPos))
        driver.x.onPress(DepositSequence(robot, 200.0, LiftConstants.lowPos, 0.3))
        driver.rightTrigger.onPress(ClawCmds.AutoOpenCmd(robot.claw, robot.guide, GuideConstants.telePos))

        gunner.leftTrigger.onPress(InstantCmd({robot.lift.setPos(-15.5)}))
        gunner.rightTrigger.onPress(InstantCmd({robot.arm.setPos(-270.0)}))
        gunner.leftBumper.onPress(InstantCmd({robot.lift.setPos(11.0)}))
        gunner.rightBumper.onPress(InstantCmd({robot.lift.setPos(0.0)}))
        gunner.b.onPress(InstantCmd({ robot.lift.setPos(2.25)}))
        gunner.a.onPress(InstantCmd({ robot.lift.setPos(3.25)}))
        gunner.x.onPress(InstantCmd({ robot.lift.setPos(4.5)}))
        gunner.y.onPress(InstantCmd({ robot.lift.setPos(5.75)}))
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
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("lift pos", robot.hardware.liftLeadMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)
        Logger.addTelemetryData("lift power", robot.hardware.liftLeadMotor.power)
        Logger.addTelemetryData("whacker pos", robot.hardware.whackerServo.position)
        Logger.addTelemetryData("claw pos", robot.hardware.clawServo.position)
        Logger.addTelemetryData("dSensor", robot.guide.lastRead)
    }
}