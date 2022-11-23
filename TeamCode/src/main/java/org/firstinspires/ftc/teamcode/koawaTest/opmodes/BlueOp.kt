package org.firstinspires.ftc.teamcode.koawaTest.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.koawaTest.Robot
import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.command.commands.*
import com.asiankoala.koawalib.logger.LoggerConfig

@TeleOp
@Config
class BlueOp : KOpMode(photonEnabled = false) {
    private val startPose = Pose(0.0, 0.0, 0.0.radians)
    lateinit var robot : Robot

    override fun mInit() {
        robot = Robot(startPose)
        robot.drive.defaultCommand = MecanumCmd(
            robot.drive,
            driver.leftStick,
            driver.rightStick.xInverted,
            0.9,
            0.9,
            0.9,
            1.0,
            1.0,
            1.0
        )

        Logger.config = LoggerConfig.DASHBOARD_CONFIG

        driver.x.onPress(InstantCmd({ driver.rumbleBlips(3) }))
        driver.y.onPress(InstantCmd({ driver.rumble(2500) }))

    }

    override fun mLoop() {

    }
}
