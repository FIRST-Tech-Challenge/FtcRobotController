package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.leftFlyWheel
import org.firstinspires.ftc.teamcode.Variables.rightFlyWheel

@TeleOp(name = "FlyWheelTest", group = "Flywheel")
class FlyWheelTest: DriveMethods() {
    override fun runOpMode() {


        leftFlyWheel = hardwareMap.get(CRServo::class.java, "leftFlyWheel")
        rightFlyWheel = hardwareMap.get(CRServo::class.java, "rightFlyWheel")
        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.b) {
                leftFlyWheel?.setPower(-10.0)
                rightFlyWheel?.setPower(10.0)
            }
        }
    }
}