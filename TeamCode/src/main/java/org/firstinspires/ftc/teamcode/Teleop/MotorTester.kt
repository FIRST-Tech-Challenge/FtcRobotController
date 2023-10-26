package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor

@TeleOp(name = "MotorTester", group = "Testing")
class MotorTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        var motorBeingTested = slideRotationMotor

        waitForStart()

        while (opModeIsActive()) {
            motorBeingTested?.power = gamepad2.left_stick_y.toDouble()/4

            telemetry.addData("Motor Value: ", motorBeingTested?.currentPosition)
            telemetry.update()
        }
    }
}