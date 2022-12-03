package org.firstinspires.ftc.teamcode.koawaTest.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.koawalib.Robot
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
            driver.rightStick,
            0.9,
            0.9,
            0.9,
            1.0,
            1.0,
            1.0
        )

        Logger.config = LoggerConfig.DASHBOARD_CONFIG

//        driver.dpadUp.onPress(Lift.LiftMove(1.0, robot.lift))
//        driver.dpadDown.onPress(Lift.LiftMove(-0.4, robot.lift))
//        driver.dpadUp.onRelease(Lift.LiftMove(0.0, robot.lift))
//        driver.dpadDown.onRelease(Lift.LiftMove(0.0, robot.lift))
//
//        driver.leftBumper.onPress(Arm.ArmMove(1.0, robot.arm))
//        driver.rightBumper.onPress(Arm.ArmMove(-0.6, robot.arm))
//        driver.leftBumper.onRelease(Arm.ArmMove(0.0, robot.arm))
//        driver.rightBumper.onRelease(Arm.ArmMove(0.0, robot.arm))
    }

    override fun mLoop() {
        Logger.addTelemetryData("drive powers", robot.drive.powers)
    }
}
