package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor

@Disabled
@TeleOp(name = "RotationMotor", group = "Testing")
class RotationTest: DriveMethods() {
    override fun runOpMode() {
        //var motorBeingTested = hardwareMap.get<DcMotor>(DcMotor::class.java, "slideRotationMotor")
        var motorBeingTested = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideLeft")

        motorBeingTested!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorBeingTested!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorBeingTested!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        waitForStart()

        while (opModeIsActive()) {
            motorBeingTested.targetPosition = ((motorBeingTested!!.currentPosition) + gamepad2.left_stick_y.toDouble()*10).toInt()
            if (gamepad2.left_stick_y >0) {
                motorBeingTested.power = -.3
                //motorBeingTested.power = .05
            }
            else if (gamepad2.left_stick_y <0){
                motorBeingTested.power = .3
                //motorBeingTested.power = -.05
            }
            else {
                motorBeingTested.power = 0.0
            }

            telemetry.addData("Motor Value: ", motorBeingTested.currentPosition)
            telemetry.update()
        }
    }
}