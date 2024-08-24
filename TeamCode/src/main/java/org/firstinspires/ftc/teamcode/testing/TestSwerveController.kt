package org.firstinspires.ftc.teamcode.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.supers.Robot

class TestSwerveController : LinearOpMode() {
    override fun runOpMode() {
        val robot: Robot = Robot(this)

        waitForStart()

        while (opModeIsActive()) {

            robot.driveController.xSpeedPower = gamepad1.left_stick_x.toDouble()
            robot.driveController.ySpeedPower = gamepad1.left_stick_y.toDouble()
            robot.driveController.rotationPower = gamepad1.right_stick_x.toDouble()

            robot.driveController.updateModules(robot.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))
        }
    }
}