package org.firstinspires.ftc.teamcode.testing.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.testing.Robot
import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.command.commands.*
import com.asiankoala.koawalib.control.filter.SlewRateLimiter
import com.asiankoala.koawalib.logger.LoggerConfig
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.DriveHackCmd

@TeleOp
@Config
class BlueOp : KOpMode(photonEnabled = false) {
    private val startPose = Pose(-48.0, -48.0, 90.0.radians)
    lateinit var robot : Robot

    override fun mInit() {
        robot = Robot(startPose)
        robot.drive.defaultCommand = MecanumCmd(
            robot.drive,
            driver.leftStick,
            driver.rightStick,
            0.9,
            0.9,
            0.9,
            1.0,
            1.0,
            1.0
        )
//        driver.leftStick.setXRateLimiter(SlewRateLimiter(0.9))
//        driver.leftStick.setYRateLimiter(SlewRateLimiter(0.9))
//        driver.rightStick.setXRateLimiter(SlewRateLimiter(0.9))

        Logger.config = LoggerConfig.DASHBOARD_CONFIG

        driver.x.onPress(InstantCmd({ driver.rumbleBlips(3) }))
        driver.y.onPress(InstantCmd({ driver.rumble(2500) }))

//        val path = PathBuilder(startPose.toPose2d())
//            .splineTo(Vector2d(-24.0, -36.0), 45.0.radians)
//            .splineTo(Vector2d(-14.0, 0.0), 90.0.radians)
//            .build()
//
//        driver.a.onPress(
//            GVFCmd(
//                robot.drive,
//                path,
//                0.7,
//                1.0 / 25.0,
//                4.0,
//                0.8,
//                2.0
//            )
//        )
    }

    override fun mLoop() {
//        Logger.addTelemetryData("arm angle", robot.arm.motor.pos)
//        Logger.addTelemetryData("is at target", robot.arm.motor.isAtTarget())
//        Logger.addVar("target vel", robot.arm.motor.setpoint.v)
//        Logger.addVar("curr vel", robot.arm.motor.currState.v)
        Logger.addTelemetryData("toggled space", driver.leftTrigger.isToggled)
        Logger.addTelemetryData("toggled aimbot", driver.a.isToggled)
    }
}
