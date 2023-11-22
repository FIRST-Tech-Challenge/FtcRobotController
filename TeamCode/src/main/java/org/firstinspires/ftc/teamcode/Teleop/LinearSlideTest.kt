package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import kotlin.math.exp

@TeleOp(name = "LSTest", group = "LinearSlide")
class LinearSlideTest: DriveMethods() {
    override fun runOpMode() {

        initSlideMotors()
        //initVision(Variables.VisionProcessors.APRILTAG)

        waitForStart()
        var speed = 400;
        var max = 1.0;
        var target = 0.0
        motorSlideLeft!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        sleep(100)
        motorSlideLeft!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

            var Pos = motorSlideLeft?.let { -(it.currentPosition) }


            var joystickY= -gamepad1.left_stick_y;
//            var joystickX = gamepad1.left_stick_x;



            if (target < Pos!!) {
                speed = 300
                max = 0.8
            } else {
                speed = 300
                max = 1.0
            }
            motorSlideLeft?.power = -((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max
            if (gamepad1.x) {
                sleep(200)
            }
//
            if (gamepad1.a&& target<850) {

                    target += 50

                sleep(200)
            }
            if (gamepad1.b && target>25) {
                target -= if(target<100){
                    25
                }else{
                    50
                }

                sleep(200)
            }
            if (gamepad1.y) {

                    target = 500.0

                sleep(200)
            }
            if (gamepad1.x) {
                    target = 850.0

                sleep(200)
            }
            if (gamepad1.left_bumper) {
                target = 0.0
                sleep(200)
            }
            if(target+joystickY<850&&target+joystickY>25) {
                var increase = if (joystickY<0) {
                    joystickY*50;
                }else{
                    joystickY * 100;
                }

                target += increase;
                sleep(70)
            }else if(target+joystickY>=850){
                target=850.0;
                sleep(70);
            }else if(target+joystickY<=25){
                target=25.0;
                sleep(70);
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