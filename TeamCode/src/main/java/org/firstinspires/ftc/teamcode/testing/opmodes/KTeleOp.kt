package asiankoala.testing.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import asiankoala.testing.Robot
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.asiankoala.koawalib.command.commands.*
import com.asiankoala.koawalib.logger.LoggerConfig

@TeleOp
@Config
class KTeleOp : KOpMode() {
    private val startPose = Pose(-36.0, -60.0, 90.0.radians)
    private val robot by lazy { Robot(startPose) }

    override fun mInit() {
        robot.drive.setDefaultCommand(
            MecanumCmd(
                robot.drive,
                driver.leftStick,
                driver.rightStick,
                0.5,
                0.5,
                0.5,
                1.0,
                1.0,
                1.0
            )
        )

        Logger.config = LoggerConfig.DASHBOARD_CONFIG

        driver.x.onPress(InstantCmd({ driver.rumbleBlips(3) }))
        driver.y.onPress(InstantCmd({ driver.rumble(2500) }))

        val path = PathBuilder(startPose.toPose2d())
            .splineTo(Vector2d(-24.0, -36.0), 45.0.radians)
            .splineTo(Vector2d(-14.0, 0.0), 90.0.radians)
            .build()

        driver.a.onPress(
            GVFCmd(
                robot.drive,
                path,
                0.7,
                1.0 / 25.0,
                4.0,
                0.9,
                2.0
            )
        )
    }

    override fun mLoop() {
//        Logger.addTelemetryData("arm angle", robot.arm.motor.pos)
//        Logger.addTelemetryData("is at target", robot.arm.motor.isAtTarget())
//        Logger.addVar("target vel", robot.arm.motor.setpoint.v)
//        Logger.addVar("curr vel", robot.arm.motor.currState.v)
    }
}
