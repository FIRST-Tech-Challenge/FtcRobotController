package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.encoders
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.motorSlideRight
import org.firstinspires.ftc.teamcode.Variables.rotationsPerMeter
import org.firstinspires.ftc.teamcode.Variables.slideLength
import org.firstinspires.ftc.teamcode.Variables.slideToBoard
import org.firstinspires.ftc.teamcode.Variables.t
import kotlin.math.E
import kotlin.math.abs
import kotlin.math.exp

@TeleOp(name = "LSTest", group = "LinearSlide")
class LinearSlideTest: DriveMethods() {
    override fun runOpMode() {

        initSlideMotors()
        initVision(Variables.VisionProcessors.APRILTAG)

        waitForStart()
        var speed = 400;
        var max = 1.0;
        var target = 0.0
        var posY = 0.0
        var closeToBoard = false;
        motorSlideLeft!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        sleep(100)
        motorSlideLeft!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

            var Pos = motorSlideLeft?.let { -(it.currentPosition) }
            if (target < Pos!!) {
                speed = 50
                max = 1.0
            } else {
                speed = 300
                max = 1.0
            }
            motorSlideLeft?.power = -((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max
            if (gamepad1.x) {
                if (!closeToBoard) {
                    closeToBoard = true
                    for (detection in aprilTag.getDetections()) {
                        // Original source data
                        posY = detection.rawPose.y;
                        slideToBoard = posY + .05
                    }
                } else {
                    !closeToBoard
                }
                sleep(200)
            }
            if (gamepad1.a && target<1600) {
                if (closeToBoard) {
                    t = .05
                    linearSlideCalc()
                    target = (slideLength / 3.3) * encoders
                } else {
                    target += 100
                }
                sleep(200)
            }
            if (gamepad1.b && target>+0) {
                if (closeToBoard) {
                    t -= .05
                    linearSlideCalc()
                    target = (slideLength / 3.3) * encoders
                } else {
                    target -= 100
                }
                sleep(200)
            }
            if (gamepad1.y) {
                if (closeToBoard) {
                    t += .26
                    linearSlideCalc()
                    target = (slideLength / 3.3) * encoders
                } else {
                    target = 500.0
                }
                sleep(200)
            }
            if (gamepad1.x) {
                if (closeToBoard) {
                    t += .78
                    linearSlideCalc()
                    target = (slideLength / 3.3) * encoders
                } else {
                    target = 1500.0
                }
                sleep(200)
            }
            if (gamepad1.left_bumper) {
                target = 0.0
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