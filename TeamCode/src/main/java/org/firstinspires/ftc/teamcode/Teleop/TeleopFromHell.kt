package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.bottom
import org.firstinspires.ftc.teamcode.Variables.clawMotor
import org.firstinspires.ftc.teamcode.Variables.clawRotation
import org.firstinspires.ftc.teamcode.Variables.closedClaw
import org.firstinspires.ftc.teamcode.Variables.high
import org.firstinspires.ftc.teamcode.Variables.lMax
import org.firstinspires.ftc.teamcode.Variables.lMin
import org.firstinspires.ftc.teamcode.Variables.lPower
import org.firstinspires.ftc.teamcode.Variables.low
import org.firstinspires.ftc.teamcode.Variables.mid
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.openClaw
import org.firstinspires.ftc.teamcode.Variables.rMax
import org.firstinspires.ftc.teamcode.Variables.rMin
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.rPower
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor
import org.firstinspires.ftc.teamcode.Variables.speed
import kotlin.math.exp

@TeleOp(name = "TeleopFromHell", group = "TeleopFinal")
class TeleopFromHell: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot() //init rack and pinion & wheel motors
        initSlideMotors() //init claw/slide motors

        telemetry.addLine("good luck buddy")
        telemetry.update()

        waitForStart()
        //set claw position into bounds
        clawRotation!!.position = 0.3
        clawMotor!!.position = closedClaw
        //reset motors
        motorSlideLeft!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorSlideLeft!!.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        //set up variables
        var leftY: Double
        var leftYGPadTwo: Double
        var leftX: Double
        var rightX: Double
        var upOrDown = "Up"
        var slideTarget = bottom
        var targetHeight = 0
        var slidePos = 0
        var speedDiv = 1.66
        while (opModeIsActive()) {
            //set gamepad inputs
            leftY = (-gamepad1.left_stick_y).toDouble()
            leftYGPadTwo = (-gamepad2.left_stick_y).toDouble()
            leftX = gamepad1.left_stick_x.toDouble()
            rightX = gamepad1.right_stick_x.toDouble()

            //set motor speeds
            slideRotationMotor?.power = leftYGPadTwo //not confirmed, could be wrong direction
            motorFL?.power = (leftY + leftX + rightX) / speedDiv
            motorBL?.power = (leftY - leftX + rightX) / speedDiv
            motorFR?.power = (leftY - leftX - rightX) / speedDiv
            motorBR?.power = (leftY + leftX - rightX) / speedDiv

            //open/close claw
            if (gamepad2.b) {
                clawMotor!!.position = closedClaw
            }
            if (gamepad2.a) {
                clawMotor!!.position = openClaw
            }

            //raise/lower slide
            if (gamepad2.left_trigger >= 0.8) {
                if (targetHeight < 3) {
                    targetHeight++
                }
                sleep(150)
            }
            if (gamepad2.left_bumper) {
                if (targetHeight > 0) {
                    targetHeight--
                }
                sleep(150)
            }

            when (targetHeight) {
                0 -> {slideTarget = bottom}
                1 -> {slideTarget = low}
                2 -> {slideTarget = mid}
                3 -> {slideTarget = high}
            }

            slidePos = motorSlideLeft?.let { -(it.currentPosition) }!!
            speed = if (slideTarget < slidePos!!) {
                50
            } else {
                300
            }
            motorSlideLeft?.power = -((2 / (1 + (exp((-(slideTarget - slidePos) / speed).toDouble())))) - 1)

            //rack & pinion control
            if (gamepad2.right_bumper) {
                if (upOrDown == "Up") {
                    if (rMotorL?.currentPosition!! < lMax) {
                        rMotorL!!.power = lPower
                    }
                } else {
                    if (rMotorL?.currentPosition!! > lMin) {
                        rMotorL!!.power = -lPower
                    }
                }
            }
            if (gamepad2.right_trigger >= 0.5) {
                if (upOrDown == "Up") {
                    if (rMotorR?.currentPosition!! < rMax) {
                        rMotorR!!.power = rPower
                    }
                } else {
                    if (rMotorR?.currentPosition!! > rMin) {
                        rMotorR!!.power = -rPower
                    }
                }
            }
            if (gamepad2.x) {
                if (upOrDown == "Up") {
                    upOrDown == "Down"
                } else {
                    upOrDown == "Up"
                }
            }

            //claw stuff
            if (gamepad2.y) {
                //swap between intake and place claw rotation
            }
        }
    }
}