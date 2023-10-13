package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.motorSlideRight
import kotlin.math.E
import kotlin.math.exp

@TeleOp(name = "LSTest", group = "LinearSlide")
class LinearSlideTest: DriveMethods() {
    override fun runOpMode() {

        initSlideMotors()


        waitForStart()
        var speed = 400;
        var max = 1.0;
        var target = 0.0;
        motorSlideLeft!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        sleep(100)
        motorSlideLeft!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

            var Pos = motorSlideLeft?.let { Math.abs(it.currentPosition) }
            if (target < Pos!!) {
                speed = 50
                max = 0.7ro
            } else {
                speed = 300
                max = 1.0
            }
            motorSlideLeft?.power = -((2 / (1 + (exp(-(target - Pos!!) / speed)))) - 1) * max
            if (gamepad1.x) {
                //here is cool stuff
                sleep(200)
            }
            if (gamepad1.a) {
                target+=100
                sleep(200)
            }
            if (gamepad1.b) {
                target -= 100
                sleep(200)
            }
            if (gamepad1.y) {
                target = 500.0
                sleep(200)
            }
            if (gamepad1.x) {
                target = 1500.0
                sleep(200)
            }
            if (true) {
                telemetry.addData("Target:", target)
                telemetry.addData("Position: ", Pos)
                telemetry.addData("Speed: ", speed)
                telemetry.addData("Power", motorSlideLeft?.power)
                telemetry.update()
            }
        }
    }
}