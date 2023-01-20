package teamcode.mule.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.mule.MuleRobot
import teamcode.mule.commands.MuleArmCommands
import teamcode.mule.constants.MuleArmConstants

@TeleOp
open class MuleOp() : KOpMode(photonEnabled = false) {
    private val robot by lazy { MuleRobot() }

    override fun mInit() {
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleTest()
    }



    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(MuleArmConstants.highPos)}, robot.arm))
        driver.rightBumper.onPress(MuleArmCommands(robot.arm, MuleArmConstants.homeWaitPos, -50.0))
    }

    override fun mLoop() {
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)

    }
}