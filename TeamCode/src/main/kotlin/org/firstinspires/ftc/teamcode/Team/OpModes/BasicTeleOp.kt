package org.firstinspires.ftc.teamcode.Team.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot
import kotlin.math.abs

@TeleOp(name = "Basic Testing Mecanum TeleOp")
class BasicTeleOp : LinearOpMode() {
    var robot = Robot()
    override fun runOpMode() {
        robot.init(hardwareMap)
        waitForStart()

        //drive controls
        var accel: Double
        var rotate: Double
        var powR: Double
        var powL: Double
        while (opModeIsActive()) {
            accel = gamepad1.left_stick_y.toDouble()
            rotate = gamepad1.left_stick_x.toDouble()
            //Determines ratio of motor powers (by sides) using the right stick
            val rightRatio = 0.5 - 0.5 * rotate
            val leftRatio = 0.5 + 0.5 * rotate
            //Declares the maximum power any side can have
            var maxRatio = 1.0
            //If we're turning left, the right motor should be at maximum power, so it decides the maxRatio. If we're turning right, vice versa.
            maxRatio = if (rotate < 0) {
                1 / rightRatio
            } else {
                1 / leftRatio
            }
            //Uses maxRatio to max out the motor ratio so that one side is always at full power.
            powR = rightRatio * maxRatio
            powL = leftRatio * maxRatio
            //Uses left trigger to determine slowdown.

            // variables used in lambda functions *MUST* be marked as final
            val powRight = -powR * accel
            val powLeft = -powL * accel
            if (gamepad1.left_bumper or gamepad1.right_bumper) {
                robot.pivotTurn(1.0, gamepad1.left_bumper, gamepad1.right_bumper)
            } else if ((abs(gamepad1.right_stick_x) > 0.5) or (abs(gamepad1.right_stick_y) > 0.5)) {
                robot.octoStrafe(-0.7, gamepad1.right_stick_y > 0.5, gamepad1.right_stick_y < -0.5, gamepad1.right_stick_x > 0.5, gamepad1.right_stick_x < -0.5)
            } else if (abs(gamepad1.left_stick_y) > 0.1) {
                robot.mapRight { m: DcMotor -> m.power = powRight }
                robot.mapLeft { m: DcMotor -> m.power = powLeft }
            } else {
                robot.stop()
            }
            telemetry.update()
        }
    }
}