package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor

@TeleOp(name = "MotorTester", group = "Testing")
class MotorTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        var motorBeingTested = slideRotationMotor

        motorBeingTested!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        waitForStart()

        while (opModeIsActive()) {
            motorBeingTested?.power = gamepad2.left_stick_y.toDouble()/4

            telemetry.addData("Motor Value: ", motorBeingTested?.currentPosition)
            telemetry.update()
        }
    }
}