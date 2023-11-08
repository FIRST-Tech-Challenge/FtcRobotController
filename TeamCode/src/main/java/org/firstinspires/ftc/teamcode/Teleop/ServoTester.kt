package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.clawMotor
import org.firstinspires.ftc.teamcode.Variables.slideGate

@TeleOp(name = "ServoTester", group = "Testing")
class ServoTester: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        initSlideMotors()
        var ServoBeingTested = hardwareMap.get(Servo::class.java, "passiveServo")

        waitForStart()

        ServoBeingTested?.position = -0.2
        while (opModeIsActive()) {
            if (gamepad2.y) {
                ServoBeingTested?.position = ServoBeingTested?.position?.plus(0.1)!!
                sleep(150)
            }

            if (gamepad2.x) {
                ServoBeingTested?.position = ServoBeingTested?.position?.minus(0.1)!!
                sleep(150)
            }
            telemetry.addData("Servo Value: ", ServoBeingTested?.position)
            telemetry.update()
        }
    }
}