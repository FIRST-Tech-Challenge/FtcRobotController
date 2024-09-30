package org.firstinspires.ftc.teamcode.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.supers.Robot

@TeleOp(name = "Test Controller")
class TestSwerveController : LinearOpMode() {
    override fun runOpMode() {
        val robot: Robot = Robot(this)

        waitForStart()

        while (opModeIsActive()) {

            robot.driveController.xSpeedPower = gamepad1.left_stick_x.toDouble()
            robot.driveController.ySpeedPower = gamepad1.left_stick_y.toDouble()
            robot.driveController.rotationPower = gamepad1.right_stick_x.toDouble()

            robot.driveController.updateModules(robot.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

            robot.dashboardTelemetry.addData("XS", robot.driveController.xSpeedPower)
            robot.dashboardTelemetry.addData("YS", robot.driveController.ySpeedPower)
            robot.dashboardTelemetry.addData("RS", robot.driveController.rotationPower)

            robot.dashboardTelemetry.addData("LmT", robot.driveController.leftModule.top.power)
            robot.dashboardTelemetry.addData("LmB", robot.driveController.leftModule.bottom.power)
            robot.dashboardTelemetry.addData("RmT", robot.driveController.rightModule.top.power)
            robot.dashboardTelemetry.addData("RmB", robot.driveController.rightModule.bottom.power)

            robot.dashboardTelemetry.addData("LmAngle", robot.driveController.leftModule.getAngle())
            robot.dashboardTelemetry.addData("RmAngle", robot.driveController.rightModule.getAngle())

            robot.dashboardTelemetry.update()
        }
    }
}