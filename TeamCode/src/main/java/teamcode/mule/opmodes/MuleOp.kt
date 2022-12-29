package teamcode.mule.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.mule.MuleRobot

@TeleOp
open class MuleOp() : KOpMode(photonEnabled = true) {
    private val robot by lazy { MuleRobot() }

    override fun mInit() {
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleTest()
    }



    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(135.0)}, robot.arm))
//        driver.leftBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.openPos)}))
//        driver.rightBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.closePos)}))
        driver.a.onPress(InstantCmd({robot.arm.setPos(-50.0)}, robot.arm))
    }

    override fun mLoop() {
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)

    }
}