package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.clawMotor

@TeleOp(name = "ServoTester", group = "Testing")
class ServoTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        var ServoBeingTested = clawMotor

        while (opModeIsActive()) {
            ServoBeingTested?.position = ServoBeingTested?.position?.plus(gamepad2.left_stick_y)!!
            telemetry.addData("Servo Value: ", ServoBeingTested?.position)
            telemetry.update()
        }
    }
}